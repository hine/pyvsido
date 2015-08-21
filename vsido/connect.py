# coding:utf-8
'''
Python3用V-Sido Connectライブラリ

Copyright (c) 2015 Daisuke IMAI

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
'''
import sys
import time
import threading
import types
import serial

DEFAULT_BAUTRATE = 115200

class Connect(object):
    '''
    V-Sido CONNECTのためのクラス
    '''

    _COMMAND_ST = 0xff;
    _COMMAND_OP_ANGLE = 0x6f # 'o'
    _COMMAND_OP_COMPLIANCE = 0x63 # 'c'
    _COMMAND_OP_MIN_MAX = 0x6d # 'm'
    _COMMAND_OP_SERVO_INFO = 0x64 # 'd'
    _COMMAND_OP_FEEDBACK_ID = 0x66 # 'f'
    _COMMAND_OP_GET_FEEDBACK = 0x72 # 'r'
    _COMMAND_OP_SET_VID_VALUE = 0x73 # 's'
    _COMMAND_OP_GET_VID_VALUE = 0x67 # 'g'
    _COMMAND_OP_WRITE_FLASH = 0x77 # 'w'
    _COMMAND_OP_GPIO = 0x69 # 'i'
    _COMMAND_OP_PWM = 0x70 # 'p'
    _COMMAND_OP_CHECK_SERVO = 0x6a # 'j'
    _COMMAND_OP_IK = 0x6b # 'k'
    _COMMAND_OP_WALK = 0x74 # 't'
    _COMMAND_OP_ACCELERATION = 0x61 # 'a'
    _COMMAND_OP_ACK = 0x21 # '!'

    def __init__(self, debug=False):
        '''
        初期化処理

        インスタンス生成に伴う処理

        Args:
            debug デバグフラグ(省略可、省略した場合はデバッグモードはOFF)
        Returns:
            なし
        Raises:
            なし
        '''
        # デバグモード設定
        self._debug = debug

        # 受信用のバッファ用意
        self._receive_buffer = []
        self._response_waiting_buffer = []

        # 送受信後に呼び出される関数の初期設定
        self._post_receive_process = self._post_receive
        self._post_send_process = self._post_send

        self._reset_values()

    def _reset_values(self):
        ''' V-Sido CONNECT接続後に取得データのクリア '''
        self._connected = False
        self._firmware_version = None
        self._pwm_cycle = None

    def set_post_receive_process(self, post_receive_process):
        '''
        レスポンス一式受信後の処理を定義

        シリアル通信でレスポンス一式を受け取った後に個別の処理を追加したい場合、この関数を利用し定義する。
        定義する関数は、引数としてレスポンスのリストを取れるようにしてなければならない。

        Args:
            post_receive_process 受信後実行する関数
        Returns:
            なし
        Raises:
            ConnectParameterError 引数に変数以外が渡された場合発生
        '''
        if isinstance(post_receive_process, types.FunctionType):
            self._post_receive_process = post_receive_process
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)

    def set_post_send_process(self, post_send_process):
        '''
        コマンド一式送信後の処理を定義

        シリアル通信でコマンド一式を送った後に個別の処理を追加したい場合、この関数を利用し定義する。
        定義する関数は、引数としてレスポンスのリストを取れるようにしてなければならない。

        Args:
            post_send_process 送信後実行する関数
        Returns:
            なし
        Raises:
            ConnectParameterError 引数に変数以外が渡された場合発生
        '''
        if isinstance(post_send_process, types.FunctionType):
            self._post_send_process = post_send_process
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)

    def _post_receive(self, received_data):
        ''' 受信後処理のダミー関数 '''
        if self._debug:
            received_data_str = []
            for data in received_data:
                received_data_str.append('%02x' % data)
            print('[debug]< ' + ' '.join(received_data_str))

    def _post_send(self, sent_data):
        ''' 送信後処理のダミー関数 '''
        if self._debug:
            sent_data_str = []
            for data in sent_data:
                sent_data_str.append('%02x' % data)
            print('[debug]> ' + ' '.join(sent_data_str))

    def connect(self, port, baudrate=DEFAULT_BAUTRATE):
        '''
        V-Sido CONNECTにシリアルポート経由で接続

        シリアルポートを通じてV-Sido CONNECTに接続する。
        色々なコマンドを投げる前にまず実行しなければならない。

        Args:
            port シリアルポート文字列(Example: 'COM3' '/dev/tty.usbserial' )
            baudrate 通信速度(省略可)
        Returns:
            なし
        Raises:
            serial.SerialException シリアルポートがオープンできなかった場合発生
        '''
        try:
            self._serial = serial.serial_for_url(port, baudrate, timeout=1)
        except serial.SerialException:
            sys.stderr.write('could not open port %r: %s\n' % (port, e))
            raise
        self._connected = True
        self._start_receiver()
        while self._firmware_version is None:
            try:
                self._firmware_version = self.get_vid_version(timeout=5)
            except ConnectTimeoutError:
                pass

    def disconnect(self):
        '''
        V-Sido CONNECTからの切断

        V-Sido CONNECTと接続しているシリアルポートを明示的に閉じ切断する。
        シリアルポートは通常はプログラムが終了した時に自動的に閉じる。

        Args:
            なし
        Returns:
            なし
        Raises:
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not self._connected:
            raise ConnectNotConnectedError(sys._getframe().f_code.co_name)
        self._stop_receiver()
        self._serial.close()
        self._reset_values()

    def _start_receiver(self):
        ''' 受信スレッドの立ち上げ '''
        self._receiver_alive = True
        self._receiver_thread = threading.Thread(target=self._receiver)
        self._receiver_thread.setDaemon(True)
        self._receiver_thread.start()

    def _stop_receiver(self):
        ''' 受信スレッドの停止 '''
        self._receiver_alive = False
        self._receiver_thread.join()

    def _receiver(self):
        ''' 受信スレッドの処理 '''
        try:
            while self._receiver_alive:
                data = self._serial.read(1)
                if len(data) > 0:
                    if int.from_bytes(data, byteorder='big') == Connect._COMMAND_ST:
                        self._receive_buffer = []
                    self._receive_buffer.append(int.from_bytes(data, byteorder='big'))
                    if len(self._receive_buffer) > 3:
                        if len(self._receive_buffer) == self._receive_buffer[2]:
                            if not self._receive_buffer[1] == Connect._COMMAND_OP_ACK:
                                # ackじゃなかった場合はレスポンス待ちのデータということで格納する
                                self._response_waiting_buffer = self._receive_buffer
                            self._post_receive_process(self._receive_buffer)
                            self._receive_buffer = []
        except serial.SerialException:
            self.alive = False
            raise

    def set_servo_angle(self, angle_data_set, cycle_time):
        '''
        V-Sido CONNECTに「目標角度設定」コマンドの送信

        各サーボモータに目標角度情報を与える。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数の角度範囲は-180度～180度だが、実際の可動域はロボットによる。
        目標角度に移行するまでの時間の引数はmsec単位で指定できるが、精度は10msec。

        Args:
            angle_data_set サーボの角度情報を書いた辞書データのリスト
                sid サーボID
                angle 角度(範囲は-180.0～180.0度、精度は0.1度)
                example:
                [{'sid':1, 'angle':20}, {'sid':2, 'angle':-20}]
            cycle_time 目標角度に移行するまでの時間(範囲は0～1000msec)
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(angle_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for angle_data in angle_data_set:
            if 'sid' in angle_data:
                if isinstance(angle_data['sid'], int):
                    if not 1 <= angle_data['sid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'angle' in angle_data:
                if isinstance(angle_data['angle'], int) or isinstance(angle_data['angle'], float):
                    if not -180.0 <= angle_data['angle'] <= 180.0:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        if isinstance(cycle_time, int):
            if not 0 <= cycle_time <= 1000:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_servo_angle_command(angle_data_set, cycle_time))

    def _make_set_servo_angle_command(self, angle_data_set, cycle_time):
        ''' 「目標角度設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_ANGLE) # OP
        data.append(0x00) # LN仮置き
        data.append(round(cycle_time / 10)) # CYC(引数はmsec単位で来るが、データは10msec単位で送る)
        for angle_data in angle_data_set:
            data.append(angle_data['sid']) # SID
            angle_data = self.make_2byte_data(round(angle_data['angle'] * 10))
            data.append(angle_data[0]) # ANGLE
            data.append(angle_data[1]) # ANGLE
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_servo_compliance(self, compliance_data_set):
        '''
        V-Sido CONNECTに「コンプライアンス設定」コマンドの送信

        各サーボモータのコンプライアンスに関する設定を行う。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数のコンプライアンススロープ値の範囲は1~254。

        Args:
            compliance_data_set サーボのコンプライアンス情報を書いた辞書データのリスト
                sid サーボID
                compliance_cw 時計回りのコンプライアンススロープ値(範囲は1～254)
                compliance_ccw 反時計回りのコンプライアンススロープ値(範囲は1～254)
                example:
                [{'sid':1, 'compliance_cw':100, 'compliance_ccw':100}, {'sid':2, 'compliance_cw':100, 'compliance_ccw':50}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(compliance_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for compliance_data in compliance_data_set:
            if 'sid' in compliance_data:
                if isinstance(compliance_data['sid'], int):
                    if not 1 <= compliance_data['sid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'compliance_cw' in compliance_data:
                if isinstance(compliance_data['compliance_cw'], int):
                    if not 1 <= compliance_data['compliance_cw'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'compliance_ccw' in compliance_data:
                if isinstance(compliance_data['compliance_ccw'], int):
                    if not 1 <= compliance_data['compliance_ccw'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_servo_compliance_command(compliance_data_set))

    def _make_set_servo_compliance_command(self, compliance_data_set):
        ''' 「コンプライアンス設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_COMPLIANCE) # OP
        data.append(0x00) # LN仮置き
        for compliance_data in compliance_data_set:
            data.append(compliance_data['sid']) # SID
            data.append(compliance_data['compliance_cw']) # CP1
            data.append(compliance_data['compliance_ccw']) # CP2
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_servo_min_max_angle(self, min_max_data_set):
        '''
        V-Sido CONNECTに「最大・最小角設定」コマンドの送信

        各サーボモータの可動範囲の最大、最小角度を設定する。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数の角度範囲は-180度～180度だが、実際の可動域はロボットによる。

        Args:
            min_max_data_set サーボの最大最小角度情報を書いた辞書データのリスト(範囲は-180.0～180.0度)
                sid サーボID
                min 最小角度(範囲は-180.0～180.0度、精度は0.1度)
                max 最大角度(範囲は-180.0～180.0度、精度は0.1度)
                example:
                [{'sid':1, 'min':-100, 'max':100}, {'sid':2, 'min':-150, 'max':50}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(min_max_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for min_max_data in min_max_data_set:
            if 'sid' in min_max_data:
                if isinstance(min_max_data['sid'], int):
                    if not 0 <= min_max_data['sid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'min' in min_max_data:
                if isinstance(min_max_data['min'], int) or isinstance(min_max_data['min'], float):
                    if not -180.0 <= min_max_data['min'] <= 180.0:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'max' in min_max_data:
                if isinstance(min_max_data['max'], int) or isinstance(min_max_data['max'], float):
                    if not -180.0 <= min_max_data['max'] <= 180.0:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if min_max_data['max'] < min_max_data['min']:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_servo_min_max_angle_command(min_max_data_set))

    def _make_set_servo_min_max_angle_command(self, min_max_data_set):
        ''' 「最大・最小角設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_MIN_MAX) # OP
        data.append(0x00) # LN仮置き
        for min_max_data in min_max_data_set:
            data.append(min_max_data['sid']) # SID
            angle_data = self.make_2byte_data(round(min_max_data['min'] * 10))
            data.append(angle_data[0]) # MIN
            data.append(angle_data[1]) # MIN
            angle_data = self.make_2byte_data(round(min_max_data['max'] * 10))
            data.append(angle_data[0]) # MAX
            data.append(angle_data[1]) # MAX
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_servo_info(self, servo_data_set, timeout=1):
        '''
        V-Sido CONNECTに「サーボ情報要求」コマンドを送信

        サーボの現在情報を取得する。
        複数のサーボへの要求をまとめて送ることができる。
        取得するサーボ情報は、開始アドレスと取得したいデータ長を決める。

        Args:
            servo_data_set サーボ情報を書いた辞書データのリスト
                sid サーボID
                address サーボ情報格納先先頭アドレス
                length サーボ情報読み出しデータ長
                example:
                [{'sid':3, 'address':1, 'length':20}, {'sid':4, 'address':1, 'length':20]
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            サーボ現在情報を書いた辞書データのリスト(引数servo_data_setにdataを加えたもの)
                example:
                [{'sid':3, 'address':1, 'length':2, 'data':[0x01, 0x02]}, {'sid':4, 'address':1, 'length':2, 'data':[0x01, 0x02]]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if not isinstance(servo_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for servo_data in servo_data_set:
            if 'sid' in servo_data:
                if isinstance(servo_data['sid'], int):
                    if not 0 <= servo_data['sid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'address' in servo_data:
                if isinstance(servo_data['address'], int):
                    if not 0 <= servo_data['address'] <= 53:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'length' in servo_data:
                if isinstance(servo_data['length'], int):
                    if not 1 <= servo_data['length'] <= 54:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        return self._parse_servo_info_response(servo_data_set, self._send_data_wait_response(self._make_get_servo_info_command(servo_data_set), timeout))

    def _make_get_servo_info_command(self, servo_data_set):
        ''' 「サーボ情報要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_SERVO_INFO) # OP
        data.append(0x00) # LN仮置き
        for servo_data in servo_data_set:
            data.append(servo_data['sid']) # SID
            data.append(servo_data['address']) # DAD
            data.append(servo_data['length']) # DLN
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_servo_info_response(self, servo_data_set, response_data):
        ''' 「サーボ情報要求」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if len(response_data) < 6:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_SERVO_INFO:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        data_pos = 3;
        for i in range(0, len(servo_data_set)):
            if not response_data[data_pos] == servo_data_set[i]['sid']:
                raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
            data_pos += 1
            servo_data = []
            for j in range(0, servo_data_set[i]['length']):
                servo_data.append(response_data[data_pos + j])
            servo_data_set[i]['data'] = servo_data
            data_pos += servo_data_set[i]['length']
        return servo_data_set

    def set_feedback_id(self, sid_data_set):
        '''
        V-Sido CONNECTに「フィードバックID設定」コマンドの送信

        フィードバック情報を欲しいサーボモータのIDの設定を行う。
        フィードバックIDを設定した後、フィードバック要求を行うこと。

        Args:
            sid_data_set サーボの最大最小角度情報を書いた辞書データのリスト(範囲は-180.0～180.0度)
                sid サーボID
                example:
                [{'sid':1}, {'sid':2}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(sid_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for sid_data in sid_data_set:
            if 'sid' in sid_data:
                if isinstance(sid_data['sid'], int):
                    if not 0 <= sid_data['sid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_feedback_id_command(sid_data_set))

    def _make_set_feedback_id_command(self, sid_data_set):
        ''' 「フィードバックID設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_FEEDBACK_ID) # OP
        data.append(0x00) # LN仮置き
        for sid_data in sid_data_set:
            data.append(sid_data['sid']) # SID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_servo_feedback(self, address, length, timeout=1):
        '''
        V-Sido CONNECTに「フィードバック要求」コマンドを送信

        フィードバックID設定で設定したサーボの現在情報を取得する。

        Args:
            address サーボ情報格納先先頭アドレス(範囲は0～53)
            length サーボ情報読み出しデータ長(範囲は1～54)
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            サーボ現在情報を書いた辞書データのリスト
                example:
                [{'sid':3, 'address':1, 'length':2, 'data':[0x01, 0x02]}, {'sid':4, 'address':1, 'length':2, 'data':[0x01, 0x02]]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if isinstance(address, int):
            if not 0 <= address <= 53:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if isinstance(length, int):
            if not 1 <= length <= 54:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        return self._parse_servo_feedback_response(address, length, self._send_data_wait_response(self._make_get_servo_feedback_command(address, length), timeout))

    def _make_get_servo_feedback_command(self, address, length):
        ''' 「サーボ情報要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GET_FEEDBACK) # OP
        data.append(0x00) # LN仮置き
        data.append(address) # DAD
        data.append(length) # DLN
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_servo_feedback_response(self, address, length, response_data):
        ''' 「サーボ情報要求」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if len(response_data) < 6:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_GET_FEEDBACK:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        servo_num = (len(response_data) - 4) // (length + 1)
        servo_data_set = []
        for i in range(0, servo_num):
            servo_data = {}
            servo_data['sid'] = response_data[3 + i * length]
            servo_data['address'] = address
            servo_data['length'] = length
            servo_data['data'] = []
            for j in range(0, length):
                servo_data['data'].append(response_data[4 + i * length + j])
            servo_data_set.append(servo_data)
        return servo_data_set

    def set_vid_io_mode(self, gpio_data_set):
        '''
        GPIOピン4～7番を入出力どちらで利用するかのVID設定の書き込み

        GPIOピンの4～7番を入出力に使うかの設定を行う。

        Args:
            gpio_data_set GIPOの設定を書いた辞書データのリスト
                iid GPIOのピン番号(範囲は4～7)
                mode 0が入力、1が出力
                example:
                [{'iid':4, 'mode':0}, {'iid':5, 'mode':1}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(gpio_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        mode_data = 0;
        for gpio_data in gpio_data_set:
            if 'iid' in gpio_data:
                if isinstance(gpio_data['iid'], int):
                    if not gpio_data['iid'] in [4, 5, 6, 7]:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'mode' in gpio_data:
                if isinstance(gpio_data['mode'], int):
                    if not gpio_data['mode'] in [0, 1]:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            mode_data += (2 ** (gpio_data['iid'] - 1)) * gpio_data['mode']
        self.set_vid_value([{'vid':3, 'vdt':mode_data}])

    def set_vid_use_pwm(self, use=True):
        '''
        PWM利用を利用するかどうかのVID設定の書き込み

        GPIOピンの6番、7番をPWM出力に使うかの設定を行う。
        引数を省略した場合はPWMを利用することとなる。

        Args:
            use GPIOピン6番、7番をPWMに使うかどうかのbool値(省略可)
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(use, bool):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if use:
            self.set_vid_value([{'vid':5, 'vdt':1}])
            if self._pwm_cycle is None:
                self._pwm_cycle = self.get_vid_pwm_cycle()
        else:
            self.set_vid_value([{'vid':5, 'vdt':0}])

    def set_vid_pwm_cycle(self, pwm_cycle):
        '''
        PWM周期を設定するVID設定の書き込み

        GPIOピンの6番、7番のPWM出力のPWM周期の設定を行う。
        引数を省略した場合はPWMを利用することとなる。

        Args:
            pwm_cycle PWM周期(範囲は4～16384usecだが、精度は4usec)
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(pwm_cycle, int):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if not 4 <= pwm_cycle <= 65536: # 16384 * 4まで
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        pwm_cycle_data = round(pwm_cycle / 4)
        # vdt = self.make_2byte_data # TODO(hine.gdw@gmail.com):本来はこれが正しいがver.2.2時点ではバグによりこうなっていない
        vdt = [pwm_cycle_data // 256, pwm_cycle_data % 256]
        self.set_vid_value([{'vid':6, 'vdt':vdt[0]}, {'vid':7, 'vdt':vdt[1]}])
        self._pwm_cycle = pwm_cycle

    def set_vid_value(self, vid_data_set):
        '''
        V-Sido CONNECTに「VID設定」コマンドの送信

        各種変数を保持するVIDの設定の書き込みを行う。
        複数のVIDの情報をまとめて送ることができる。
        VIDの定義内容はコマンドリファレンス参照。

        Args:
            vid_data_set VID設定情報を書いた辞書データのリスト
                vid 設定値ID
                vdt 設定値
                example:
                [{'vid':6, 'vdt':0x0e}, {'vid':7, 'vdt':0xa6}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(vid_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for vid_data in vid_data_set:
            if 'vid' in vid_data:
                if isinstance(vid_data['vid'], int):
                    # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認などに対応
                    if not 0 <= vid_data['vid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'vdt' in vid_data:
                if isinstance(vid_data['vdt'], int) or isinstance(vid_data['vdt'], float):
                    # 2Byteデータの取り扱いについては仕様書を要確認
                    if not 0 <= vid_data['vdt'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_vid_value_command(vid_data_set))

    def _make_set_vid_value_command(self, vid_data_set):
        ''' 「VID設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_SET_VID_VALUE) # OP
        data.append(0x00) # LN仮置き
        for vid_data in vid_data_set:
            data.append(vid_data['vid']) # VID
            data.append(vid_data['vdt']) # VDT(※2Byteになるデータがある模様だが、それぞれでIDふられているので、LISTに入れるようにすること)
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_vid_version(self, timeout=1):
        '''
        バージョン情報のVID設定の取得

        V-Sido CONNECTのバージョン情報をVID設定から読み取る。

        Args:
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            バージョンを示す数値
        Raises:
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        return self.get_vid_value([{'vid':254}], timeout)[0]['vdt']

    def get_vid_pwm_cycle(self, timeout=1):
        '''
        PWM周期のVID設定の取得

        PWM周期の情報をVID設定から読み取る。

        Args:
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            PWM周期を示す数値(VID格納値の4倍)
        Raises:
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        pwd_data = self.get_vid_value([{'vid':6}, {'vid':7}], timeout)
        return (pwd_data[0]['vdt'] * 256 + pwd_data[1]['vdt']) * 4

    def get_vid_value(self, vid_data_set, timeout=1):
        '''
        V-Sido CONNECTに「VID要求」コマンドを送信

        各種変数を保持するVIDの設定の読み込みを行う。
        複数のVIDの要求をまとめて送ることができる。
        VIDの定義内容はコマンドリファレンス参照。

        Args:
            vid_data_set VID設定情報を書いた辞書データのリスト
                example:
                vid 設定値ID
                [{'vid':6}, {'vid':7}]
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            VID設定情報を書いた辞書データのリスト(引数vid_data_setにvdtを加えたもの)
                vid 設定値ID
                vdt 設定値
                example:
                [{'vid':6, 'vdt':0x0e}, {'vid':7, 'vdt':0xa6}]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if not isinstance(vid_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for vid_data in vid_data_set:
            if 'vid' in vid_data:
                if isinstance(vid_data['vid'], int):
                    # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認などに対応
                    if not 0 <= vid_data['vid'] <= 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        return self._parse_vid_response(vid_data_set, self._send_data_wait_response(self._make_get_vid_value_command(vid_data_set), timeout))

    def _make_get_vid_value_command(self, vid_data_set):
        ''' 「VID要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GET_VID_VALUE) # OP
        data.append(0x00) # LN仮置き
        for vid_data in vid_data_set:
            data.append(vid_data['vid']) # VID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_vid_response(self, vid_data_set, response_data):
        ''' 「VID要求」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if len(response_data) < 5:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_GET_VID_VALUE:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        vid_num = len(response_data) - 4 # TODO(hine.gdw@gmail.com):ver.2.2現在バグで0x00が多くついてくる(下で0x00を許容しているので、ここはこれでOK)
        if not len(vid_data_set) == vid_num:
            if len(vid_data_set) == vid_num - 1: # TODO(hine.gdw@gmail.com):仮に00がついていてもOKなロジックとする
                if response_data[3 + len(vid_data_set)] == 0x00:
                    vid_num -= 1
                else:
                    raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        for i in range(0, vid_num):
            vid_data_set[i]['vdt'] = response_data[3 + i]
        return vid_data_set

    def write_flash	(self):
        '''
        V-Sido CONNECTに「フラッシュ書き込み要求」コマンドの送信

        現在のVID設定情報をV-Sido CONNECTのフラッシュに書き込む。

        Args:
            なし
        Returns:
            なし
        Raises:
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        self._send_data(self._make_write_flash_command())

    def _make_write_flash_command(self):
        ''' 「フラッシュ書き込み要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_WRITE_FLASH) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_gpio_config	(self, gpio_data_set):
        '''
        V-Sido CONNECTに「IO設定」コマンドの送信

        GPIOピン4～7番の出力を設定する。
        事前にset_vid_set_io_mode()でピンを出力に切り替えなければならない。
        valueは0でLOW、1でHIGHとなる。

        Args:
            gpio_data_set 出力情報を書いた辞書データのリスト
                iid GPIOピン番号(範囲は4～7)
                value 0がLOW、1がHIGH
                example:
                [{'iid':4, 'value':1}, {'iid':5, 'value':0}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(gpio_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for gpio_data in gpio_data_set:
            if 'iid' in gpio_data:
                if isinstance(gpio_data['iid'], int):
                    if not gpio_data['iid'] in [4, 5, 6, 7]:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'value' in gpio_data:
                if isinstance(gpio_data['value'], int):
                    if not gpio_data['value'] in [0, 1]:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_gpio_config_command(gpio_data_set))

    def _make_set_gpio_config_command(self, gpio_data_set):
        ''' 「IO設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GPIO) # OP
        data.append(0x00) # LN仮置き
        for gpio_data in gpio_data_set:
            data.append(gpio_data['iid']) # VID
            data.append(gpio_data['value']) # VAL
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_pwm_pulse_width(self, pwm_data_set):
        '''
        V-Sido CONNECTに「PWM設定」コマンドの送信

        GPIOピン6番、7番にPWMのパルス幅を設定する。
        事前にset_vid_use_pwm()でPWMを利用できるようにしなければならない。
        パルス幅の上限は、VID設定のPWM周期以下でなければならない。

        Args:
            pwm_data_set パルス幅情報を書いた辞書データのリスト
                example:
                iid GPIOピン番号(範囲は6～7)
                pulse パルス幅(範囲は0～65536で4usec単位)
                [{'iid':6, 'pulse':15000}, {'iid':7, 'pulse':7500}]
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if not isinstance(pwm_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if self._pwm_cycle is None:
            self._pwm_cycle = self.get_vid_pwm_cycle()
        for pwm_data in pwm_data_set:
            if 'iid' in pwm_data:
                if isinstance(pwm_data['iid'], int):
                    if not pwm_data['iid'] in [6, 7]:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'pulse' in pwm_data:
                if isinstance(pwm_data['pulse'], int):
                    if not 0 <= pwm_data['pulse'] <= self._pwm_cycle:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_set_pwm_pulse_width_command(pwm_data_set))

    def _make_set_pwm_pulse_width_command(self, pwm_data_set):
        ''' 「PWM設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_PWM) # OP
        data.append(0x00) # LN仮置き
        for pwm_data in pwm_data_set:
            data.append(pwm_data['iid']) # VID
            pulse_data = self.make_2byte_data(round(pwm_data['pulse'] / 4))
            data.append(pulse_data[0]) # ANGLE
            data.append(pulse_data[1]) # ANGLE
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def check_connected_servo(self, timeout=1):
        '''
        V-Sido CONNECTに「接続確認要求」コマンドを送信

        サーボモータの接続確認を行う。

        Args:
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            サーボ接続情報を書いた辞書データのリスト
                sid サーボID
                time 関節角度受信までの時間(usec)
                example:
                [{'sid':6, 'time':48}, {'sid':7, 'time':50}]
        Raises:
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        return self._parse_check_servo_response(self._send_data_wait_response(self._make_check_connected_servo_command(), timeout))

    def _make_check_connected_servo_command(self):
        ''' 「接続確認要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_CHECK_SERVO) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_check_servo_response(self, response_data):
        ''' 「接続確認要求」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if len(response_data) < 6:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_CHECK_SERVO:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        sid_num = (len(response_data) - 4) // 2
        sid_data_set = []
        for i in range(0, sid_num):
            sid_data = {}
            sid_data['sid'] = response_data[3 + i * 2]
            sid_data['time'] = response_data[4 + i * 2]
            sid_data_set.append(sid_data)
        return sid_data_set

    def set_ik(self, ik_data_set, feedback=False, timeout=0.5):
        '''
        V-Sido CONNECTに「IK設定」コマンドの送信

        IK(Inverse Kinematics:逆運動学)に基づいた手足の位置指定を行う。
        手先、足先位置を、x:左右、y:前後、z:上下で指定することで、関節角度の計算が行われる。

        Args:
            ik_data_set IK設定情報を書いた辞書データのリスト
                kid IK部位の番号
                kdt IK用設定データ
                    x x座標に関するデータ
                    y y座標に関するデータ
                    z z座標に関するデータ
                example:
                [{'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}}]
            feedback コマンド送信後、IK情報のリターンを求めるかのbool値(省略可、省略した場合フィードバックなし)
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は0.5秒)
        Returns:
            現在のIK位置の辞書データのリスト(ただし、引数でfeedback=Trueの場合のみ)
                kid IK部位の番号
                kdt IK用設定データ
                    x x座標に関するデータ
                    y y座標に関するデータ
                    z z座標に関するデータ
                example:
                [{'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}}]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if not isinstance(ik_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for ik_data in ik_data_set:
            if 'kid' in ik_data:
                if isinstance(ik_data['kid'], int):
                    if not 0 <= ik_data['kid'] <= 15:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
            if 'kdt' in ik_data:
                if isinstance(ik_data['kdt'], dict):
                    if 'x' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['x'], int):
                            if not -180 <= ik_data['kdt']['x'] <= 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                    if 'y' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['y'], int):
                            if not -180 <= ik_data['kdt']['y'] <= 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                    if 'z' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['z'], int):
                            if not -180 <= ik_data['kdt']['z'] <= 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                else:
                    raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        if not isinstance(feedback, bool):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if not feedback:
            self._send_data(self._make_set_ik_command(ik_data_set, feedback))
        else:
            return self._parse_ik_response(self._send_data_wait_response(self._make_set_ik_command(ik_data_set, feedback)))

    def _make_set_ik_command(self, ik_data_set, feedback):
        ''' 「IK設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_IK) # OP
        data.append(0x00) # LN仮置き
        if not feedback:
            data.append(0x01) # IKF
        else:
            data.append(0x09) # IKF
        for ik_data in ik_data_set:
            data.append(ik_data['kid']) # KID
            data.append(ik_data['kdt']['x'] + 100) # KDT_X
            data.append(ik_data['kdt']['y'] + 100) # KDT_Y
            data.append(ik_data['kdt']['z'] + 100) # KDT_Z
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_ik(self, ik_data_set, timeout=1):
        '''
        V-Sido CONNECTに「IK取得」コマンドの送信

        IK(Inverse Kinematics:逆運動学)に基づいた手足の位置情報取得。

        Args:
            ik_data_set IK設定情報を書いた辞書データのリスト
                kid IK部位の番号
                example:
                [{'kid':2}, {'kid':3}]
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            現在のIK位置の辞書データのリスト(引数ik_data_setにkdtを付加したもの)
                kid IK部位の番号
                kdt IK用設定データ
                    x x座標に関するデータ
                    y y座標に関するデータ
                    z z座標に関するデータ
                example:
                [{'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}}]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if not isinstance(ik_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        for ik_data in ik_data_set:
            if 'kid' in ik_data:
                if isinstance(ik_data['kid'], int):
                    if not 0 <= ik_data['kid'] <= 15:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        return self._parse_ik_response(self._send_data_wait_response(self._make_get_ik_command(ik_data_set, feedback), timeout))

    def _make_get_ik_command(self, ik_data_set):
        ''' 「IK取得」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_IK) # OP
        data.append(0x00) # LN仮置き
        data.append(0x08) # IKF
        for ik_data in ik_data_set:
            data.append(ik_data['kid']) # KID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_ik_response(self, response_data):
        ''' 「IK設定」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if len(response_data) < 9:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_IK:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        ik_num = (len(response_data) - 5) // 4
        ik_data_set = []
        for i in range(0, ik_num):
            ik_data = {}
            ik_data['kid'] = response_data[i * 4 + 4]
            ik_data['kdt'] = {}
            ik_data['kdt']['x'] = response_data[i * 4 + 5] - 100
            ik_data['kdt']['y'] = response_data[i * 4 + 6] - 100
            ik_data['kdt']['z'] = response_data[i * 4 + 7] - 100
            ik_data_set.append(ik_data)
        return ik_data_set

    def walk(self, forward, turn_cw):
        '''
        V-Sido CONNECTに「移動情報指定（歩行）」コマンドの送信

        ロボットを歩行させる。最後にコマンドを受け取ってからおよそ3秒で停止する。
        歩行中も次のコマンドを受け取ることができる。

        Args:
            forward 前後の移動方向(-100～100で前が正)
            turn_cw 左右の旋回方向(-100～100で時計回りが正)
        Returns:
            なし
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
        '''
        if isinstance(forward, int):
            if not -100 <= forward <= 100:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if isinstance(turn_cw, int):
            if not -100 <= turn_cw <= 100:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        self._send_data(self._make_walk_command(forward, turn_cw))

    def _make_walk_command(self, forward, turn_cw):
        ''' 「移動情報指定（歩行）」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_WALK) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # WAD(Utilityでは0で固定)
        data.append(0x02) # WLN(現在2で固定)
        # 速度ならびに旋回は-100〜100を0〜200に変換する
        data.append(forward + 100)
        data.append(turn_cw + 100)
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_acceleration(self, timeout=1):
        '''
        V-Sido CONNECTに「加速度センサー値要求」コマンドの送信

        V-Sido CONNECTに接続した加速度センサーの値を取得する。
        VID設定で接続したセンサーを正しく設定していなければならない。

        Args:
            timeout 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)
        Returns:
            加速度センサー値の辞書データ(引数ik_data_setにkdtを付加したもの)
                ax X軸方向の加速度(1~253)
                ay Y軸方向の加速度(1~253)
                az Z軸方向の加速度(1~253)
                example:
                {'ax': 125, 'az': 118, 'ay': 158}
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
            ConnectNotConnectedError 接続していなかった場合発生
            ConnectTimeoutError レスポンスがタイムアウトした場合発生
            ConnectInvalidResponseError レスポンスが期待する書式と違った場合発生
        '''
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        return self._parse_acceleration_response(self._send_data_wait_response(self._make_get_acceleration_command(), timeout))

    def _make_get_acceleration_command(self):
        ''' 「加速度センサ値要求」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_ACCELERATION) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_acceleration_response(self, response_data):
        ''' 「加速度センサ値要求」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if not len(response_data) == 7:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        if not response_data[1] == Connect._COMMAND_OP_ACCELERATION:
            raise ConnectInvalidResponseError(sys._getframe().f_code.co_name)
        acceleration_data = {}
        acceleration_data['ax'] = response_data[3]
        acceleration_data['ay'] = response_data[4]
        acceleration_data['az'] = response_data[5]
        return acceleration_data

    def _send_data(self, command_data):
        ''' V-Sido CONNECTにシリアル経由でデータ送信 '''
        if not self._connected:
            raise ConnectNotConnectedError(sys._getframe().f_code.co_name)
        data_bytes = b''
        for data in command_data:
            data_bytes += data.to_bytes(1, byteorder='little')
        self._serial.write(data_bytes)
        self._post_send_process(command_data)

    def _send_data_wait_response(self, command_data, timeout=0.5):
        ''' V-Sido CONNECTにシリアル経由でデータ送信して受信を待つ '''
        self._response_waiting_buffer = []
        try:
            self._send_data(command_data)
        except ConnectNotConnectedError:
            raise
        wait_start = time.time()
        while not self._response_waiting_buffer:
            if not timeout == 0:
                if time.time() - wait_start > timeout:
                    raise ConnectTimeoutError(sys._getframe().f_code.co_name)
        return self._response_waiting_buffer

    def make_2byte_data(self, value):
        '''
        数値データから2Byteデータを作る

        データ部に0xffが入ることを防ぐために2Byteのデータを0xffを含まない形に変形する
         (「V-Sido CONNECT RC Command Reference」参照)

        Args:
            value 加工したい数値
        Returns:
            加工済み2Byteのデータのリスト
                example:
                [0x0e, 0xa6]
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
        '''
        if not isinstance(value, int):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        value_bytes = value.to_bytes(2, byteorder='little', signed=True)
        # 上位バイトを左に1ビットシフト
        value_high_tmp = (value_bytes[1] << 1) & 0x00ff
        # 再び数値に戻し、全体を左に1ビットシフトする
        value_tmp = (int.from_bytes([value_bytes[0], value_high_tmp], byteorder='little', signed=True) << 1)
        value_tmp_bytes = value_tmp.to_bytes(2, byteorder='little', signed=True)
        return [value_tmp_bytes[0], value_tmp_bytes[1]]

    def parse_2byte_data(self, data):
        '''
        V-Sido CONNECTからの2Byteデータを数値に戻す

        データ部に0xffが入ることを防ぐために2Byteのデータは0xffを含まない形に変形されているので、
        それを元の数値に戻す
         (「V-Sido CONNECT RC Command Reference」参照)

        Args:
            data 加工したい2Byteのデータのリスト
                example:
                [0x0e, 0xa6]
        Returns:
            加工して元に戻した数値
        Raises:
            ConnectParameterError 引数の条件を間違っていた場合発生
        '''
        if not isinstance(data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        if not len(data) == 2:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
        # データを数値に戻し、全体を右に1ビットシフトする
        value_tmp = int.from_bytes(data, byteorder='little', signed=True) >> 1
        # 再度、上位Byteと下位Byteに分解する
        value_tmp_bytes = value_tmp.to_bytes(2, byteorder='big', signed=True)
        # 上位バイトだけ右に1ビットシフトする
        value_high = (value_tmp_bytes[0] & 0x80) | (value_tmp_bytes[0] >> 1)
        value_low = value_tmp_bytes[1]
        return int.from_bytes([value_high, value_low], byteorder='big', signed=True)

    def _adjust_ln_sum(self, command_data):
        ''' データ中のLN(Length)とSUM(CheckSum)の調整 '''
        ln_pos = 1 if command_data[0] in [0x0c, 0x0d, 0x53, 0x54] else 2
        if len(command_data) > 3:
            command_data[ln_pos] = len(command_data);
            sum = 0;
            for data in command_data:
                sum ^= data
            command_data[len(command_data) - 1] = sum
            return command_data


class ConnectParameterError(Exception):
    ''' V-Sido CONNECT Command Parameter Error '''
    pass


class ConnectNotConnectedError(Exception):
    ''' V-Sido CONNECT Not Connected Error '''
    pass


class ConnectTimeoutError(Exception):
    ''' V-Sido CONNECT Timeout Error '''
    pass


class ConnectInvalidResponseError(Exception):
    ''' V-Sido CONNECT Invalid Response Error '''
    pass


if __name__ == '__main__':

    import sys
    import time

    DEFAULT_PORT = '/dev/tty.usbserial'

    # 受信データを表示する関数のサンプル
    def post_receive(received_data):
        received_data_str = []
        for data in received_data:
            received_data_str.append('%02x' % data)
        print('< ' + ' '.join(received_data_str))

    # 送信データを表示する関数のサンプル
    def post_send(sent_data):
        sent_data_str = []
        for data in sent_data:
            sent_data_str.append('%02x' % data)
        print('> ' + ' '.join(sent_data_str))

    print('=== Python V-Sido TEST ===')

    # 引数からシリアルポートを決定する
    if len(sys.argv) == 1:
        port = DEFAULT_PORT
    else:
        port = sys.argv[1]
    baudrate = DEFAULT_BAUTRATE

    # V-Sido CONNECT用のインスタンス生成（初期化でシリアルポートをオープンする）
    vsidoconnect = Connect()
    # シリアルポートをオープン、受信スレッドの立ち上げ
    print('Connecting to robot...', end='')
    try:
        vsidoconnect.connect(port, baudrate)
    except serial.SerialException:
        print('fail.')
        sys.exit(1)
    print('done.')

    # 送受信後の処理を自作関数に置き換える方法
    vsidoconnect.set_post_send_process(post_send)
    vsidoconnect.set_post_receive_process(post_receive)
    print('exit: Ctrl-C')
    print('')

    # テストで歩行コマンド
    vsidoconnect.walk(100, 0)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        vsidoconnect.disconnect()
