# coding:utf-8
'''Python3用V-Sido Connectライブラリ

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
    '''V-Sido CONNECTのためのクラス
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

    def __init__(self, post_receive_handler=None, post_send_handler=None, debug=False):
        '''初期化処理

        インスタンス生成に伴う処理

        Args:
            post_receive_handler(function/method): 受信後実行する関数
            post_send_handler(function/method): 送信後実行する関数
            debug(Optional[bool]): debag(送受信の履歴表示)モードはTrue、そうでない時はFalseを指定

        Raises:
            ValueError: invalid argument
        '''
        # デバグモード設定
        if not isinstance(debug, bool):
            raise ValueError('debug must be bool')
        self._debug = debug

        # 送受信後に呼び出される関数の初期設定
        if post_receive_handler is not None:
            if not (isinstance(post_receive_handler, types.FunctionType) or isinstance(post_receive_handler, types.MethodType)):
                raise ValueError('receive_broadcast_handler must be function or method')
        if post_send_handler is not None:
            if not (isinstance(post_send_handler, types.FunctionType) or isinstance(post_send_handler, types.MethodType)):
                raise ValueError('receive_sensor_update_handler must be function or method')
        self._post_receive_handler = post_receive_handler or self._default_post_receive_handler
        self._post_send_handler = post_send_handler or self._default_post_send_handler

        # 受信用のバッファ用意
        self._receive_buffer = []
        self._response_waiting_buffer = []

        # 接続状態などの保持値をクリア
        self._reset_values()

    def _reset_values(self):
        '''V-Sido CONNECT接続後に取得すべきデータのクリア
        '''
        self._connected = False
        self._firmware_version = None
        self._pwm_cycle = None

    def _default_post_receive_handler(self, received_data):
        '''受信後処理のデフォルト関数
        '''
        if self._debug:
            received_data_str = []
            for data in received_data:
                received_data_str.append('%02x' % data)
            print('[debug]< ' + ' '.join(received_data_str))

    def _default_post_send_handler(self, sent_data):
        '''送信後処理のデフォルト関数
        '''
        if self._debug:
            sent_data_str = []
            for data in sent_data:
                sent_data_str.append('%02x' % data)
            print('[debug]> ' + ' '.join(sent_data_str))

    def connect(self, port, baudrate=DEFAULT_BAUTRATE):
        '''V-Sido CONNECTにシリアルポート経由で接続

        シリアルポートを通じてV-Sido CONNECTに接続する。
        色々なコマンドを投げる前にまず実行しなければならない。

        Args:
            port(str): シリアルポート文字列
                Example: 'COM3', '/dev/tty.usbserial'
            baudrate(Optional[int]): 通信速度

        Raises:
            serial.SerialException: シリアルポートがオープンできなかった場合発生
        '''
        if not self._connected:
            try:
                self._serial = serial.serial_for_url(port, baudrate, timeout=1)
            except serial.SerialException:
                sys.stderr.write('could not open port %r: %s\n' % (port, e))
                raise
            self._connected = True
            self._start_receiver()
            while self._firmware_version is None:
                try:
                    self._firmware_version = self.get_vid_version(timeout=1)
                except TimeoutError:
                    pass

    def disconnect(self):
        '''V-Sido CONNECTからの切断

        V-Sido CONNECTと接続しているシリアルポートを明示的に閉じ切断する。
        シリアルポートは通常はプログラムが終了した時に自動的に閉じる。
        '''
        if self._connected:
            self._stop_receiver()
            self._serial.close()
            self._reset_values()

    def is_connected(self):
        '''V-Sidoとの接続確認

        V-Sido CONNECTと接続状態にあるかどうかの確認

        Returns:
            bool: 接続している時はTrue、接続していない時はFalseを返す
        '''
        return self._connected

    def _start_receiver(self):
        '''受信スレッドの立ち上げ
        '''
        self._receiver_alive = True
        self._receiver_thread = threading.Thread(target=self._receiver)
        self._receiver_thread.setDaemon(True)
        self._receiver_thread.start()

    def _stop_receiver(self):
        '''受信スレッドの停止
        '''
        self._receiver_alive = False
        self._receiver_thread.join()

    def _receiver(self):
        '''受信スレッドの処理
        '''
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
                            self._post_receive_handler(self._receive_buffer)
                            self._receive_buffer = []
        except serial.SerialException:
            self.alive = False
            raise

    def set_servo_angle(self, *angle_data_set, cycle_time=0):
        '''V-Sido CONNECTに「目標角度設定」コマンドの送信

        各サーボモータに目標角度情報を与える。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数の角度範囲は-180度～180度だが、実際の可動域はロボットによる。
        目標角度に移行するまでの時間の引数はmsec単位で指定できるが、精度は10msec。

        Args:
            *angle_data_set(dict): サーボの角度情報を書いた辞書データ
                sid(int): サーボID
                angle(int/float): 角度(範囲は-180.0～180.0度、精度は0.1度)
                example:
                {'sid':1, 'angle':20}, {'sid':2, 'angle':-20})
            cycle_time(Optional[int]): 目標角度に移行するまでの時間(範囲は0～1000msec)(省略した場合は0)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(cycle_time, int):
            raise ValueError('cycle_time must be int')
        if not 0 <= cycle_time <= 1000:
            raise ValueError('cycle_time must be 0 - 1000')
        for angle_data in angle_data_set:
            if not isinstance(angle_data, dict):
                raise ValueError('angle_data_set must contain dict data')
            if 'sid' not in angle_data:
                raise ValueError('missing sid in angle_data_set')
            if not isinstance(angle_data['sid'], int):
                raise ValueError('sid must be int')
            if not 1 <= angle_data['sid'] <= 254:
                raise ValueError('sid must be 1 - 254')
            if 'angle' not in angle_data:
                raise ValueError('missing angle in angle_data_set')
            if not (isinstance(angle_data['angle'], int) or isinstance(angle_data['angle'], float)):
                raise ValueError('angle must be int or float')
            if not -180.0 <= angle_data['angle'] <= 180.0:
                raise ValueError('angle must be -180 - 180')
        self._send_data(self._make_set_servo_angle_command(*angle_data_set, cycle_time=cycle_time))

    def _make_set_servo_angle_command(self, *angle_data_set, cycle_time):
        '''「目標角度設定」コマンドのデータ生成
        '''
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

    def set_servo_compliance(self, *compliance_data_set):
        '''V-Sido CONNECTに「コンプライアンス設定」コマンドの送信

        各サーボモータのコンプライアンスに関する設定を行う。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数のコンプライアンススロープ値の範囲は1~254。

        Args:
            *compliance_data_set(dict): サーボのコンプライアンス情報を書いた辞書データ
                sid(int): サーボID
                compliance_cw(int): 時計回りのコンプライアンススロープ値(範囲は1～254)
                compliance_ccw(int): 反時計回りのコンプライアンススロープ値(範囲は1～254)
                example:
                {'sid':1, 'compliance_cw':100, 'compliance_ccw':100}, {'sid':2, 'compliance_cw':100, 'compliance_ccw':50}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        for compliance_data in compliance_data_set:
            if not isinstance(compliance_data, dict):
                raise ValueError('compliance_data_set must contain dict data')
            if 'sid' not in compliance_data:
                raise ValueError('missing sid in compliance_data_set')
            if not isinstance(compliance_data['sid'], int):
                raise ValueError('sid must be int')
            if not 1 <= compliance_data['sid'] <= 254:
                raise ValueError('sid must be 1 - 254')
            if 'compliance_cw' not in compliance_data:
                raise ValueError('missing compliance_cw in compliance_data_set')
            if not isinstance(compliance_data['compliance_cw'], int):
                raise ValueError('compliance_cw must be int')
            if not 1 <= compliance_data['compliance_cw'] <= 254:
                raise ValueError('compliance_cw must be 1 - 254')
            if 'compliance_ccw' not in compliance_data:
                raise ValueError('missing compliance_ccw in compliance_data_set')
            if not isinstance(compliance_data['compliance_ccw'], int):
                raise ValueError('compliance_ccw must be int')
            if not 1 <= compliance_data['compliance_ccw'] <= 254:
                raise ValueError('compliance_ccw must be 1 - 254')
        self._send_data(self._make_set_servo_compliance_command(*compliance_data_set))

    def _make_set_servo_compliance_command(self, *compliance_data_set):
        '''「コンプライアンス設定」コマンドのデータ生成
        '''
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

    def set_servo_min_max_angle(self, *min_max_data_set):
        '''V-Sido CONNECTに「最大・最小角設定」コマンドの送信

        各サーボモータの可動範囲の最大、最小角度を設定する。
        複数のサーボモータへの情報をまとめて送ることができる。
        引数の角度範囲は-180度～180度だが、実際の可動域はロボットによる。

        Args:
            *min_max_data_set(dict): サーボの最大最小角度情報を書いた辞書データ
                sid(int): サーボID
                min(int/float): 最小角度(範囲は-180.0～180.0度、精度は0.1度)
                max(int/float): 最大角度(範囲は-180.0～180.0度、精度は0.1度)
                example:
                {'sid':1, 'min':-100, 'max':100}, {'sid':2, 'min':-150, 'max':50}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        for min_max_data in min_max_data_set:
            if not isinstance(min_max_data, dict):
                raise ValueError('min_max_data_set must contain dict data')
            if 'sid' not in min_max_data:
                raise ValueError('missing sid in min_max_data_set')
            if not isinstance(min_max_data['sid'], int):
                raise ValueError('sid must be int')
            if not 0 <= min_max_data['sid'] <= 254:
                raise ValueError('sid must be 0 - 254')
            if 'min' not in min_max_data:
                raise ValueError('missing min in min_max_data_set')
            if not (isinstance(min_max_data['min'], int) or isinstance(min_max_data['min'], float)):
                raise ValueError('min must be int or float')
            if not -180.0 <= min_max_data['min'] <= 180.0:
                raise ValueError('min must be -180 - 180')
            if 'max' not in min_max_data:
                raise ValueError('missing max in min_max_data_set')
            if not (isinstance(min_max_data['max'], int) or isinstance(min_max_data['max'], float)):
                raise ValueError('max must be int or float')
            if not -180.0 <= min_max_data['max'] <= 180.0:
                raise ValueError('max must be -180 - 180')
            if min_max_data['max'] < min_max_data['min']:
                raise ValueError('max must be bigger than min')
        self._send_data(self._make_set_servo_min_max_angle_command(*min_max_data_set))

    def _make_set_servo_min_max_angle_command(self, *min_max_data_set):
        '''「最大・最小角設定」コマンドのデータ生成
        '''
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

    def get_servo_info(self, *servo_data_set, timeout=1):
        '''
        V-Sido CONNECTに「サーボ情報要求」コマンドを送信

        サーボの現在情報を取得する。
        複数のサーボへの要求をまとめて送ることができる。
        取得するサーボ情報は、開始アドレスと取得したいデータ長を決める。

        Args:
            servo_data_set(dict): サーボ情報を書いた辞書データ
                sid(int): サーボID
                address(int): サーボ情報格納先先頭アドレス(範囲は0～53)
                length(int): サーボ情報読み出しデータ長(範囲は1～54)
                example:
                {'sid':3, 'address':1, 'length':20}, {'sid':4, 'address':1, 'length':20}
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略した場合は1秒)
        Returns:
            tuple: サーボ現在情報を書いた辞書データ(引数servo_data_setにdataを加えたもの)
                example:
                ({'sid':3, 'address':1, 'length':2, 'data':[0x01, 0x02]}, {'sid':4, 'address':1, 'length':2, 'data':[0x01, 0x02])

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        for servo_data in servo_data_set:
            if not isinstance(servo_data, dict):
                raise ValueError('servo_data_set must contain dict data')
            if 'sid' not in servo_data:
                raise ValueError('missing sid in servo_data_set')
            if not isinstance(servo_data['sid'], int):
                raise ValueError('sid must be int')
            if not 0 <= servo_data['sid'] <= 254:
                raise ValueError('sid must be 0 - 254')
            if 'address' not in servo_data:
                raise ValueError('missing address in servo_data_set')
            if not isinstance(servo_data['address'], int):
                raise ValueError('adress must be int')
            if not 0 <= servo_data['address'] <= 53:
                raise ValueError('adress must be 0 - 53')
            if 'length' not in servo_data:
                raise ValueError('missing length in servo_data_set')
            if not isinstance(servo_data['length'], int):
                raise ValueError('length must be int')
            if not 1 <= servo_data['length'] <= 54:
                raise ValueError('length must be 1 - 54')
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_servo_info_response(*servo_data_set, response_data=self._send_data_wait_response(self._make_get_servo_info_command(*servo_data_set), timeout))

    def _make_get_servo_info_command(self, *servo_data_set):
        '''「サーボ情報要求」コマンドのデータ生成
        '''
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

    def _parse_servo_info_response(self, *servo_data_set, response_data):
        '''「サーボ情報要求」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if len(response_data) < 4:
            raise ValueError('Invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_SERVO_INFO:
            raise ValueError('invalid response_data OP')
        data_pos = 3;
        for i in range(0, len(servo_data_set)):
            if not response_data[data_pos] == servo_data_set[i]['sid']:
                raise ValueError('invalid response_data')
            data_pos += 1
            servo_data = []
            for j in range(0, servo_data_set[i]['length']):
                servo_data.append(response_data[data_pos + j])
            servo_data_set[i]['data'] = servo_data
            data_pos += servo_data_set[i]['length']
        return servo_data_set

    def set_feedback_id(self, *sid_set):
        '''V-Sido CONNECTに「フィードバックID設定」コマンドの送信

        フィードバック情報を欲しいサーボモータのIDの設定を行う。
        フィードバックIDを設定した後、フィードバック要求を行うこと。

        Args:
            *sid(int): サーボID

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        for sid in sid_set:
            if not isinstance(sid, int):
                raise ValueError('sid must be int')
            if not 0 <= sid <= 254:
                raise ValueError('sid must be 1 - 254')
        self._send_data(self._make_set_feedback_id_command(*sid_set))

    def _make_set_feedback_id_command(self, *sid_set):
        '''「フィードバックID設定」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_FEEDBACK_ID) # OP
        data.append(0x00) # LN仮置き
        for sid in sid_set:
            data.append(sid) # SID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def get_servo_feedback(self, address, length, timeout=1):
        '''V-Sido CONNECTに「フィードバック要求」コマンドを送信

        フィードバックID設定で設定したサーボの現在情報を取得する。

        Args:
            address(int): サーボ情報格納先先頭アドレス(範囲は0～53)
            length(int): サーボ情報読み出しデータ長(範囲は1～54)
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略した場合は1秒)

        Returns:
            tuple: サーボ現在情報を書いた辞書データ
                example:
                ({'sid':3, 'address':1, 'length':2, 'data':[0x01, 0x02]}, {'sid':4, 'address':1, 'length':2, 'data':[0x01, 0x02]])

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        if not isinstance(address, int):
            raise ValueError('address must be int')
        if not 0 <= address <= 53:
            raise ValueError('address must be 0 - 53')
        if not isinstance(length, int):
            raise ValueError('length must be int')
        if not 1 <= length <= 54:
            raise ValueError('length must be int')
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_servo_feedback_response(address, length, response_data=self._send_data_wait_response(self._make_get_servo_feedback_command(address, length), timeout))

    def _make_get_servo_feedback_command(self, address, length):
        '''「サーボ情報要求」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GET_FEEDBACK) # OP
        data.append(0x00) # LN仮置き
        data.append(address) # DAD
        data.append(length) # DLN
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_servo_feedback_response(self, address, length, response_data):
        '''「サーボ情報要求」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if len(response_data) < 4:
            raise ValueError('Invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_GET_FEEDBACK:
            raise ValueError('Invalid response_data OP')
        servo_num = (len(response_data) - 4) // (length + 1)
        servo_data_set = tuple()
        for i in range(0, servo_num):
            servo_data = {}
            servo_data['sid'] = response_data[3 + i * length]
            servo_data['address'] = address
            servo_data['length'] = length
            servo_data['data'] = []
            for j in range(0, length):
                servo_data['data'].append(response_data[4 + i * length + j])
            servo_data_set += (servo_data,)
        return servo_data_set

    def set_vid_io_mode(self, *gpio_data_set):
        '''GPIOピン4～7番を入出力どちらで利用するかのVID設定の書き込み

        GPIOピンの4～7番を入出力に使うかの設定を行う。

        Args:
            *gpio_data_set(dict): GIPOの設定を書いた辞書データ
                iid(int): GPIOのピン番号(範囲は4～7)
                mode(int): 0が入力、1が出力
                example:
                {'iid':4, 'mode':0}, {'iid':5, 'mode':1}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        mode_data = 0;
        for gpio_data in gpio_data_set:
            if not isinstance(gpio_data, dict):
                raise ValueError('gpio_data_set must contain dict data')
            if 'iid' not in gpio_data:
                raise ValueError('missing iid in gpio_data_set')
            if not isinstance(gpio_data['iid'], int):
                raise ValueError('iid must be int')
            if not gpio_data['iid'] in [4, 5, 6, 7]:
                raise ValueError('iid must be 4 - 7')
            if 'mode' not in gpio_data:
                raise ValueError('missing mode in gpio_data_set')
            if not isinstance(gpio_data['mode'], int):
                raise ValueError('mode must be int')
            if not gpio_data['mode'] in [0, 1]:
                raise ValueError('mode must be 0 or 1')
            mode_data += (2 ** (gpio_data['iid'] - 1)) * gpio_data['mode']
        self.set_vid_value({'vid':3, 'vdt':mode_data})

    def set_vid_use_pwm(self, use=True):
        '''PWM利用を利用するかどうかのVID設定の書き込み

        GPIOピンの6番、7番をPWM出力に使うかの設定を行う。
        引数を省略した場合はPWMを利用することとなる。

        Args:
            use GPIOピン6番、7番をPWMに使うかどうかのbool値(省略可)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(use, bool):
            raise ValueError('use must be bool')
        if use:
            self.set_vid_value({'vid':5, 'vdt':1})
            if self._pwm_cycle is None:
                self._pwm_cycle = self.get_vid_pwm_cycle()
        else:
            self.set_vid_value({'vid':5, 'vdt':0})

    def set_vid_pwm_cycle(self, pwm_cycle):
        '''PWM周期を設定するVID設定の書き込み

        GPIOピンの6番、7番のPWM出力のPWM周期の設定を行う。
        引数を省略した場合はPWMを利用することとなる。

        Args:
            pwm_cycle(int): PWM周期(範囲は4～16384usecだが、精度は4usec)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(pwm_cycle, int):
            raise ValueError('pwm_cycle must be int')
        if not 4 <= pwm_cycle <= 65536: # 16384 * 4まで
            raise ValueError('pwm_cycle must be 4 - 65536')
        pwm_cycle_data = round(pwm_cycle / 4)
        # vdt = self.make_2byte_data # TODO(hine.gdw@gmail.com):本来はこれが正しいがver.2.2時点ではバグによりこうなっていない
        vdt = [pwm_cycle_data // 256, pwm_cycle_data % 256]
        self.set_vid_value({'vid':6, 'vdt':vdt[0]}, {'vid':7, 'vdt':vdt[1]})
        self._pwm_cycle = pwm_cycle

    def set_vid_value(self, *vid_data_set):
        '''V-Sido CONNECTに「VID設定」コマンドの送信

        各種変数を保持するVIDの設定の書き込みを行う。
        複数のVIDの情報をまとめて送ることができる。
        VIDの定義内容はコマンドリファレンス参照。

        Args:
            *vid_data_set(dict): VID設定情報を書いた辞書データ
                vid(int): 設定値ID
                vdt(int): 設定値
                example:
                {'vid':6, 'vdt':0x0e}, {'vid':7, 'vdt':0xa6}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        for vid_data in vid_data_set:
            if not isinstance(vid_data, dict):
                raise ValueError('vid_data_set must contain dict data')
            if 'vid' not in vid_data:
                raise ValueError('missing vid in vid_data_set')
            if not isinstance(vid_data['vid'], int):
                raise ValueError('vid must be int')
            if not 0 <= vid_data['vid'] <= 254:
                # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認(254)などに対応
                raise ValueError('vid must be 0 - 254')
            if 'vdt' not in vid_data:
                raise ValueError('missing vdt in vid_data_set')
            if not isinstance(vid_data['vdt'], int):
                raise ValueError('vdt must be int')
            if not 0 <= vid_data['vdt'] <= 254:
                # 2Byteデータの取り扱いについては仕様書を要確認
                raise ValueError('vdt must be 0 - 254')
        self._send_data(self._make_set_vid_value_command(*vid_data_set))

    def _make_set_vid_value_command(self, *vid_data_set):
        '''「VID設定」コマンドのデータ生成
        '''
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
        '''バージョン情報のVID設定の取得

        V-Sido CONNECTのバージョン情報をVID設定から読み取る。

        Args:
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self.get_vid_value(254, timeout=timeout)[0]['vdt']

    def get_vid_pwm_cycle(self, timeout=1):
        '''PWM周期のVID設定の取得

        PWM周期の情報をVID設定から読み取る。

        Args:
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        pwd_data = self.get_vid_value(6, 7, timeout=timeout)
        return (pwd_data[0]['vdt'] * 256 + pwd_data[1]['vdt']) * 4

    def get_vid_value(self, *vid_set, timeout=1):
        '''V-Sido CONNECTに「VID要求」コマンドを送信

        各種変数を保持するVIDの設定の読み込みを行う。
        複数のVIDの要求をまとめて送ることができる。
        VIDの定義内容はコマンドリファレンス参照。

        Args:
            *vid_set(int): VID設定情報
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)

        Returns:
            tuple: VID設定情報を書いた辞書データ(引数vid_data_setにvdtを加えたもの)
                vid(int): 設定値ID
                vdt(int): 設定値
                example:
                {'vid':6, 'vdt':0x0e}, {'vid':7, 'vdt':0xa6}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        for vid in vid_set:
            if not isinstance(vid, int):
                raise ValueError('vid must be int')
            if not 0 <= vid <= 254:
                # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認などに対応
                raise ValueError('vid must be int')
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_vid_response(*vid_set, response_data=self._send_data_wait_response(self._make_get_vid_value_command(*vid_set), timeout))

    def _make_get_vid_value_command(self, *vid_set):
        '''「VID要求」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GET_VID_VALUE) # OP
        data.append(0x00) # LN仮置き
        for vid in vid_set:
            data.append(vid) # VID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_vid_response(self, *vid_set, response_data):
        '''「VID要求」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if len(response_data) < 5:
            raise ValueError('invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_GET_VID_VALUE:
            raise ValueError('invalid response_data OP')
        vid_num = len(response_data) - 4 # TODO(hine.gdw@gmail.com):ver.2.2現在バグで0x00が多くついてくる(下で0x00を許容しているので、ここはこれでOK)
        if not len(vid_set) == vid_num:
            if len(vid_set) == vid_num - 1: # TODO(hine.gdw@gmail.com):仮に00がついていてもOKなロジックとする
                if response_data[3 + len(vid_set)] == 0x00:
                    vid_num -= 1
                else:
                    raise ValueError('invalid response_data')
        vid_data_set = tuple()
        for i in range(0, len(vid_set)):
            vid_data = {}
            vid_data['vid'] = vid_set[i]
            vid_data['vdt'] = response_data[3 + i]
            vid_data_set += (vid_data, )
        return vid_data_set

    def write_flash	(self):
        '''V-Sido CONNECTに「フラッシュ書き込み要求」コマンドの送信

        現在のVID設定情報をV-Sido CONNECTのフラッシュに書き込む。

        Raises:
            ConnectionError: V-Sido CONNECT is not connected
        '''
        self._send_data(self._make_write_flash_command())

    def _make_write_flash_command(self):
        '''「フラッシュ書き込み要求」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_WRITE_FLASH) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_gpio_config	(self, *gpio_data_set):
        '''V-Sido CONNECTに「IO設定」コマンドの送信

        GPIOピン4～7番の出力を設定する。
        事前にset_vid_set_io_mode()でピンを出力に切り替えなければならない。
        valueは0でLOW、1でHIGHとなる。

        Args:
            *gpio_data_set(dict) 出力情報を書いた辞書データ
                iid(int): GPIOピン番号(範囲は4～7)
                value(int): 0がLOW、1がHIGH
                example:
                {'iid':4, 'value':1}, {'iid':5, 'value':0}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        for gpio_data in gpio_data_set:
            if not isinstance(gpio_data, dict):
                raise ValueError('gpio_data_set must contain dict data')
            if 'iid' not in gpio_data:
                raise ValueError('missing iid in gpio_data_set')
            if not isinstance(gpio_data['iid'], int):
                raise ValueError('iid must be int')
            if not gpio_data['iid'] in [4, 5, 6, 7]:
                raise ValueError('iid must be 4 - 7')
            if 'value' not in gpio_data:
                raise ValueError('missing value in gpio_data_set')
            if not isinstance(gpio_data['value'], int):
                raise ValueError('value must be int')
            if not gpio_data['value'] in [0, 1]:
                raise ValueError('value must be 0 or 1')
        self._send_data(self._make_set_gpio_config_command(*gpio_data_set))

    def _make_set_gpio_config_command(self, *gpio_data_set):
        '''「IO設定」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_GPIO) # OP
        data.append(0x00) # LN仮置き
        for gpio_data in gpio_data_set:
            data.append(gpio_data['iid']) # VID
            data.append(gpio_data['value']) # VAL
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_pwm_pulse_width(self, *pwm_data_set):
        '''V-Sido CONNECTに「PWM設定」コマンドの送信

        GPIOピン6番、7番にPWMのパルス幅を設定する。
        事前にset_vid_use_pwm()でPWMを利用できるようにしなければならない。
        パルス幅の上限は、VID設定のPWM周期以下でなければならない。

        Args:
            pwm_data_set(dict): パルス幅情報を書いた辞書データ
                iid(int): GPIOピン番号(範囲は6～7)
                pulse(int): パルス幅(範囲は0～65536で4usec単位)
                example:
                {'iid':6, 'pulse':15000}, {'iid':7, 'pulse':7500}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        if self._pwm_cycle is None:
            self._pwm_cycle = self.get_vid_pwm_cycle()
        for pwm_data in pwm_data_set:
            if not isinstance(pwm_data, dict):
                raise ValueError('pwm_data_set must contain dict data')
            if 'iid' not in pwm_data:
                raise ValueError('missing iid in pwm_data')
            if not isinstance(pwm_data['iid'], int):
                raise ValueError('iid must be int')
            if not pwm_data['iid'] in [6, 7]:
                raise ValueError('iid must be 6 or 7')
            if 'pulse' not in pwm_data:
                raise ValueError('missing pulse in pwm_data')
            if not isinstance(pwm_data['pulse'], int):
                raise ValueError('pulse must be int')
            if not 0 <= pwm_data['pulse'] <= self._pwm_cycle:
                raise ValueError('pulse must be 0 - PWM_CYCLE')
        self._send_data(self._make_set_pwm_pulse_width_command(*pwm_data_set))

    def _make_set_pwm_pulse_width_command(self, *pwm_data_set):
        '''「PWM設定」コマンドのデータ生成
        '''
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
        '''V-Sido CONNECTに「接続確認要求」コマンドを送信

        サーボモータの接続確認を行う。

        Args:
            timeout(int): 受信タイムアウトするまでの秒数(省略した場合は1秒)

        Returns:
            tuple: サーボ接続情報を書いた辞書データ
                sid(int): サーボID
                time(int) 関節角度受信までの時間(usec)
                example:
                ({'sid':6, 'time':48}, {'sid':7, 'time':50})

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_check_connected_servo_response(self._send_data_wait_response(self._make_check_connected_servo_command(), timeout))

    def _make_check_connected_servo_command(self):
        '''「接続確認要求」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_CHECK_SERVO) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_check_connected_servo_response(self, response_data):
        '''「接続確認要求」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if len(response_data) < 4:
            raise ValueError('invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_CHECK_SERVO:
            raise ValueError('invalid response_data OP')
        sid_num = (len(response_data) - 4) // 2
        sid_data_set = tuple()
        for i in range(0, sid_num):
            sid_data = {}
            sid_data['sid'] = response_data[3 + i * 2]
            sid_data['time'] = response_data[4 + i * 2]
            sid_data_set += (sid_data, )
        return sid_data_set

    def set_ik(self, *ik_data_set, feedback=False, timeout=0.5):
        '''V-Sido CONNECTに「IK設定」コマンドの送信

        IK(Inverse Kinematics:逆運動学)に基づいた手足の位置指定を行う。
        手先、足先位置を、x:左右、y:前後、z:上下で指定することで、関節角度の計算が行われる。

        Args:
            *ik_data_set(dict): IK設定情報を書いた辞書データ
                kid(int): IK部位の番号
                kdt(dict): IK用設定データ
                    x(int): x座標に関するデータ
                    y(int): y座標に関するデータ
                    z(int): z座標に関するデータ
                example:
                {'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}}
            feedback(Optional[bool]): コマンド送信後、IK情報のリターンを求めるかのbool値(省略した場合フィードバックなし)
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略した場合は0.5秒)

        Returns:
            tuple: 現在のIK位置の辞書データ(ただし、引数でfeedback=Trueの場合のみ)
                kid(int): IK部位の番号
                kdt(dict): IK用設定データ
                    x(int): x座標に関するデータ
                    y(int): y座標に関するデータ
                    z(int): z座標に関するデータ
                example:
                {'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        for ik_data in ik_data_set:
            if not isinstance(ik_data, dict):
                raise ValueError('ik_data_set must contain dict data')
            if 'kid' not in ik_data:
                raise ValueError('missing kid in ik_data_set')
            if not isinstance(ik_data['kid'], int):
                raise ValueError('kid must be int')
            if not 0 <= ik_data['kid'] <= 15:
                raise ValueError('kid must be 0 - 15')
            if 'kdt' not in ik_data:
                raise ValueError('missing kdt in ik_data_set')
            if not isinstance(ik_data['kdt'], dict):
                raise ValueError('kdt must contain dict data')
            if 'x' not in ik_data['kdt']:
                raise ValueError('missing x in kdt')
            if not isinstance(ik_data['kdt']['x'], int):
                raise ValueError('x must be int')
            if not -100 <= ik_data['kdt']['x'] <= 100:
                raise ValueError('x must be -100 - 100')
            if 'y' not in ik_data['kdt']:
                raise ValueError('missing y in kdt')
            if not isinstance(ik_data['kdt']['y'], int):
                raise ValueError('y must be int')
            if not -100 <= ik_data['kdt']['y'] <= 100:
                raise ValueError('y must be -100 - 100')
            if 'z' not in ik_data['kdt']:
                raise ValueError('missing z in kdt')
            if not isinstance(ik_data['kdt']['z'], int):
                raise ValueError('z must be int')
            if not -100 <= ik_data['kdt']['z'] <= 100:
                raise ValueError('z must be -100 - 100')
        if not isinstance(feedback, bool):
            raise ValueError('feedback must be bool')
        if not feedback:
            self._send_data(self._make_set_ik_command(*ik_data_set, feedback=feedback))
        else:
            return self._parse_ik_response(self._send_data_wait_response(self._make_set_ik_command(*ik_data_set, feedback=feedback)))

    def _make_set_ik_command(self, *ik_data_set, feedback):
        '''「IK設定」コマンドのデータ生成
        '''
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

    def get_ik(self, *kid_set, timeout=1):
        '''V-Sido CONNECTに「IK取得」コマンドの送信

        IK(Inverse Kinematics:逆運動学)に基づいた手足の位置情報取得。

        Args:
            *ik_set(int): IK設定情報を書いた辞書データ
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略した場合は1秒)

        Returns:
            tuple: 現在のIK位置の辞書データ
                kid(int): IK部位の番号
                kdt(dict): IK用設定データ
                    x(int): x座標に関するデータ
                    y(int): y座標に関するデータ
                    z(int): z座標に関するデータ
                example:
                ({'kid':2, 'kdt':{'x':0, 'y':0, 'z':100}}, {'kid':3, 'kdt':{'x':0, 'y':0, 'z':100}})
        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        for kid in kid_set:
            if not isinstance(kid, int):
                raise ValueError('kid must be int')
            if not 0 <= ik_data['kid'] <= 15:
                raise ValueError('kid must be 0 - 15')
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_ik_response(response_data=self._send_data_wait_response(self._make_get_ik_command(*kid_set), timeout))

    def _make_get_ik_command(self, *kid_set):
        '''「IK取得」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_IK) # OP
        data.append(0x00) # LN仮置き
        data.append(0x08) # IKF
        for kid in kid_set:
            data.append(kid) # KID
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_ik_response(self, response_data):
        '''「IK設定」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if len(response_data) < 4:
            raise ValueError('invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_IK:
            raise ValueError('invalid response_data OP')
        ik_num = (len(response_data) - 5) // 4
        ik_data_set = tuple()
        for i in range(0, ik_num):
            ik_data = {}
            ik_data['kid'] = response_data[i * 4 + 4]
            ik_data['kdt'] = {}
            ik_data['kdt']['x'] = response_data[i * 4 + 5] - 100
            ik_data['kdt']['y'] = response_data[i * 4 + 6] - 100
            ik_data['kdt']['z'] = response_data[i * 4 + 7] - 100
            ik_data_set += (ik_data,)
        return ik_data_set

    def walk(self, forward, turn_cw):
        '''V-Sido CONNECTに「移動情報指定（歩行）」コマンドの送信

        ロボットを歩行させる。最後にコマンドを受け取ってからおよそ3秒で停止する。
        歩行中も次のコマンドを受け取ることができる。

        Args:
            forward(int): 前後の移動方向(-100～100で前が正)
            turn_cw(int): 左右の旋回方向(-100～100で時計回りが正)

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(forward, int):
            raise ValueError('forward must be int')
        if not -100 <= forward <= 100:
            raise ValueError('forward must be -100 - 100')
        if not isinstance(turn_cw, int):
            raise ValueError('turn_cw must be int')
        if not -100 <= turn_cw <= 100:
            raise ValueError('turn_cw must be -100 - 100')
        self._send_data(self._make_walk_command(forward, turn_cw))

    def _make_walk_command(self, forward, turn_cw):
        '''「移動情報指定（歩行）」コマンドのデータ生成
        '''
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
        '''V-Sido CONNECTに「加速度センサー値要求」コマンドの送信

        V-Sido CONNECTに接続した加速度センサーの値を取得する。
        VID設定で接続したセンサーを正しく設定していなければならない。

        Args:
            timeout(Optional[int/float]): 受信タイムアウトするまでの秒数(省略可、省略した場合は1秒)

        Returns:
            dict: 加速度センサー値の辞書データ
                ax(int): X軸方向の加速度(1～253)
                ay(int): Y軸方向の加速度(1～253)
                az(int): Z軸方向の加速度(1～253)
                example:
                {'ax': 125, 'az': 118, 'ay': 158}

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
            TimeoutError: V-Sido CONNECT response timeout
        '''
        if not (isinstance(timeout, int) or isinstance(timeout, float)):
            raise ValueError('timeout must be int or float')
        return self._parse_acceleration_response(response_data=self._send_data_wait_response(self._make_get_acceleration_command(), timeout))

    def _make_get_acceleration_command(self):
        '''「加速度センサ値要求」コマンドのデータ生成
        '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_ACCELERATION) # OP
        data.append(0x00) # LN仮置き
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def _parse_acceleration_response(self, response_data):
        '''「加速度センサ値要求」のレスポンスデータのパース
        '''
        if not isinstance(response_data, list):
            raise ValueError('response_data must be list')
        if not len(response_data) == 7:
            raise ValueError('invalid response_data length')
        if not response_data[1] == Connect._COMMAND_OP_ACCELERATION:
            raise ValueError('invalid response_data OP')
        acceleration_data = {}
        acceleration_data['ax'] = response_data[3]
        acceleration_data['ay'] = response_data[4]
        acceleration_data['az'] = response_data[5]
        return acceleration_data

    def _send_data(self, command_data):
        '''V-Sido CONNECTにシリアル経由でデータ送信
        '''
        if not self._connected:
            raise ConnectionError('V-Sido CONNECT is not connected')
        data_bytes = b''
        for data in command_data:
            data_bytes += data.to_bytes(1, byteorder='little')
        self._serial.write(data_bytes)
        self._post_send_handler(command_data)

    def _send_data_wait_response(self, command_data, timeout=0.5):
        '''V-Sido CONNECTにシリアル経由でデータ送信して受信を待つ
        '''
        self._response_waiting_buffer = []
        try:
            self._send_data(command_data)
        except ConnectNotConnectedError:
            raise
        wait_start = time.time()
        while not self._response_waiting_buffer:
            if not timeout == 0:
                if time.time() - wait_start > timeout:
                    raise TimeoutError('V-Sido CONNECT response timeout')
        return self._response_waiting_buffer

    def make_2byte_data(self, value):
        '''数値データから2Byteデータを作る

        データ部に0xffが入ることを防ぐために2Byteのデータを0xffを含まない形に変形する
         (「V-Sido CONNECT RC Command Reference」参照)

        Args:
            value(int): 加工したい数値

        Returns:
            list: 加工済み2Byteのデータのリスト
                example:
                [0x0e, 0xa6]

        Raises:
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(value, int):
            raise ValueError('value must be int')
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
            ValueError: invalid argument
            ConnectionError: V-Sido CONNECT is not connected
        '''
        if not isinstance(data, list):
            raise ValueError('data must be list')
        if not len(data) == 2:
            raise ValueError('invalid response_data length')
        # データを数値に戻し、全体を右に1ビットシフトする
        value_tmp = int.from_bytes(data, byteorder='little', signed=True) >> 1
        # 再度、上位Byteと下位Byteに分解する
        value_tmp_bytes = value_tmp.to_bytes(2, byteorder='big', signed=True)
        # 上位バイトだけ右に1ビットシフトする
        value_high = (value_tmp_bytes[0] & 0x80) | (value_tmp_bytes[0] >> 1)
        value_low = value_tmp_bytes[1]
        return int.from_bytes([value_high, value_low], byteorder='big', signed=True)

    def _adjust_ln_sum(self, command_data):
        '''データ中のLN(Length)とSUM(CheckSum)の調整
        '''
        ln_pos = 1 if command_data[0] in [0x0c, 0x0d, 0x53, 0x54] else 2
        if len(command_data) > 3:
            command_data[ln_pos] = len(command_data);
            sum = 0;
            for data in command_data:
                sum ^= data
            command_data[len(command_data) - 1] = sum
            return command_data


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
