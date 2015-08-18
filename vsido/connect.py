# coding:utf-8
'''
Python3用V-Sido Connectライブラリ
'''
import sys
import threading
import types
import serial

DEFAULT_BAUTRATE = 115200

class Connect(object):
    '''
    V-Sido CONNECTのためのクラス
    '''

    _COMMAND_ST = 0xff;
    _COMMAND_OP_ANGLE = 0x6f; # 'o'
    _COMMAND_OP_SET_VID_VALUE = 0x73; # 's'
    _COMMAND_OP_GET_VID_VALUE = 0x67; # 'g'
    _COMMAND_OP_PWM = 0x70; # 'p'
    _COMMAND_OP_IK = 0x6b; # 'k'
    _COMMAND_OP_WALK = 0x74; # 't'
    _COMMAND_OP_GPIO = 0x69; # 'i'

    def __init__(self):
        '''
        '''
        self._connected = False
        self._receive_buffer = []
        self._response_waiting_buffer = []
        self._firmware_version = None
        self._pwm_cycle = None
        self._post_receive_process = self._post_receive
        self._post_send_process = self._post_send

    def set_post_receive_process(self, post_receive_process):
        ''' レスポンス一式受信後の処理を定義 '''
        if isinstance(post_receive_process, types.FunctionType):
            self._post_receive_process = post_receive_process
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)

    def set_post_send_process(self, post_send_process):
        ''' コマンド一式送信後の処理を定義 '''
        if isinstance(post_send_process, types.FunctionType):
            self._post_send_process = post_send_process
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)

    def _post_receive(self, received_data):
        ''' 受信後処理のダミー '''
        pass

    def _post_send(self, sent_data):
        ''' 送信後処理のダミー '''
        pass

    def connect(self, port, baudrate=DEFAULT_BAUTRATE):
        ''' V-Sido CONNECTにシリアルポート経由で接続 '''
        if self._connected:
            return
        try:
            self._serial = serial.serial_for_url(port, baudrate, timeout=1)
        except serial.SerialException:
            sys.stderr.write("could not open port %r: %s\n" % (port, e))
            raise
        self._connected = True
        self._start_receiver()
        self._firmware_version = self.get_vid_version()

    def disconnect(self):
        ''' V-Sido CONNECTからの切断 '''
        if not self._connected:
            return
        self._stop_receiver()
        self._serial.close()
        self._connected = False

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
                    if data == b'\xff':
                        self._receive_buffer = []
                    self._receive_buffer.append(int.from_bytes(data, byteorder='big'))
                    if len(self._receive_buffer) > 3:
                        if len(self._receive_buffer) == self._receive_buffer[2]:
                            if not self._receive_buffer[1] == 0x21:
                                # ackじゃなかった場合はレスポンス待ちのデータということで格納する
                                self._response_waiting_buffer = self._receive_buffer
                            self._post_receive_process(self._receive_buffer)
                            self._receive_buffer = []
        except serial.SerialException:
            self.alive = False
            raise

    def set_servo_angle(self, angle_data_set, cycle_time):
        ''' V-Sido CONNECTに「目標確度設定」コマンドの送信 '''
        if not isinstance(angle_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        for angle_data in angle_data_set:
            if 'sid' in angle_data:
                if isinstance(angle_data['sid'], int):
                    if angle_data['sid'] < 0 or angle_data['sid'] > 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
            if 'angle' in angle_data:
                if isinstance(angle_data['angle'], int) or isinstance(angle_data['angle'], float):
                    if angle_data['angle'] < -180.0 or angle_data['angle'] > 180.0:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        if isinstance(cycle_time, int):
            if cycle_time < 0 or cycle_time > 1000:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        self._send_data(self._make_set_servo_angle_command(angle_data_set, cycle_time))

    def _make_set_servo_angle_command(self, angle_data_set, cycle_time):
        ''' 「目標確度設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_ANGLE) # OP
        data.append(0x00) # LN仮置き
        data.append(round(cycle_time / 10)) # CYC(引数はmsec単位で来るが、データは10msec単位で送る)
        for angle_data in angle_data_set:
            data.append(angle_data['sid']) # SID
            angle_data = self._make_2byte_data(round(angle_data['angle'] * 10))
            data.append(angle_data[0]) # ANGLE
            data.append(angle_data[1]) # ANGLE
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_vid_use_pwm(self, use=True):
        ''' PWM利用を利用するかどうかのVID設定の書き込み '''
        if not isinstance(use, bool):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if use:
            self.set_vid_value([{'vid':5, 'vdt':1}])
        else:
            self.set_vid_value([{'vid':5, 'vdt':0}])
        if self._pwm_cycle is None:
            self._pwm_cycle = self.get_vid_pwm_cycle()


    def set_vid_pwm_cycle(self, pwm_cycle):
        ''' PWM周期を設定するVID設定の書き込み '''
        if not isinstance(pwm_cycle, int):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if pwm_cycle < 4 or pwm_cycle > 16384:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        pwm_cycle_data = round(pwm_cycle / 4)
        # vdt = self._make_2byte_data # [Todo]:本来はこれが正しいがver.2.2時点ではバグによりこうなっていない
        vdt = [pwm_cycle_data // 256, pwm_cycle_data % 256]
        self.set_vid_value([{'vid':6, 'vdt':vdt[0]}, {'vid':7, 'vdt':vdt[1]}])
        self._pwm_cycle = pwm_cycle

    def set_vid_value(self, vid_data_set):
        ''' V-Sido CONNECTに「VID設定」コマンドの送信 '''
        if not isinstance(vid_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        for vid_data in vid_data_set:
            if 'vid' in vid_data:
                if isinstance(vid_data['vid'], int):
                    # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認などに対応
                    if vid_data['vid'] < 0 or vid_data['vid'] > 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
            if 'vdt' in vid_data:
                if isinstance(vid_data['vdt'], int) or isinstance(vid_data['vdt'], float):
                    # 2Byteデータの取り扱いについては仕様書を要確認
                    if vid_data['vdt'] < 0 or vid_data['vdt'] > 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
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

    def get_vid_version(self):
        ''' バージョン情報のVID設定の取得 '''
        return self.get_vid_value([{'vid':254}])[0]['vdt']

    def get_vid_pwm_cycle(self):
        ''' PWM周期のVID設定の取得 '''
        pwd_data = self.get_vid_value([{'vid':6}, {'vid':7}])
        return (pwd_data[0]['vdt'] * 256 + pwd_data[1]['vdt']) * 4

    def get_vid_value(self, vid_data_set):
        ''' V-Sido CONNECTに「VID要求」コマンドを送信 '''
        if not isinstance(vid_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        for vid_data in vid_data_set:
            if 'vid' in vid_data:
                if isinstance(vid_data['vid'], int):
                    # 本来はこんなに幅が広くないが将来的に拡張する可能性と、バージョン確認などに対応
                    if vid_data['vid'] < 0 or vid_data['vid'] > 254:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        return self._parse_vid_response(vid_data_set, self._send_data_wait_response(self._make_get_vid_value_command(vid_data_set)))

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
            return
        if len(response_data) < 5:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if not response_data[1] == Connect._COMMAND_OP_GET_VID_VALUE:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        vid_num = len(response_data) - 4 # [Todo]:本来は4引くだけだが、ver.2.2現在バグで0x00が多くついてくる
        if not len(vid_data_set) == vid_num:
            if len(vid_data_set) == vid_num - 1: # [Todo]:仮に00がついていてもOKなロジックとする
                if response_data[3 + len(vid_data_set)] == 0x00:
                    vid_num -= 1
                else:
                    raise ConnectParameterError(sys._getframe().f_code.co_name)
                    return
        for i in range(0, vid_num):
            vid_data_set[i]['vdt'] = response_data[3 + i]
        return vid_data_set

    def set_pwm_pulse_width(self, pwm_data_set):
        ''' V-Sido CONNECTに「PWM設定」コマンドの送信 '''
        if not isinstance(pwm_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        for pwm_data in pwm_data_set:
            if 'iid' in pwm_data:
                if isinstance(pwm_data['iid'], int):
                    if pwm_data['iid'] < 6 or pwm_data['iid'] > 7:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
            if self._pwm_cycle is None:
                self._pwm_cycle = self.get_vid_pwm_cycle()
            if 'pulse' in pwm_data:
                if isinstance(pwm_data['pulse'], int):
                    if pwm_data['pulse'] < 0 or pwm_data['pulse'] > self._pwm_cycle:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        self._send_data(self._make_set_pwm_pulse_width_command(pwm_data_set))

    def _make_set_pwm_pulse_width_command(self, pwm_data_set):
        ''' 「PWM設定」コマンドのデータ生成 '''
        data = []
        data.append(Connect._COMMAND_ST) # ST
        data.append(Connect._COMMAND_OP_PWM) # OP
        data.append(0x00) # LN仮置き
        for pwm_data in pwm_data_set:
            data.append(pwm_data['iid']) # VID
            pulse_data = self._make_2byte_data(round(pwm_data['pulse'] / 4))
            data.append(pulse_data[0]) # ANGLE
            data.append(pulse_data[1]) # ANGLE
        data.append(0x00) # SUM仮置き
        return self._adjust_ln_sum(data);

    def set_ik(self, ik_data_set, feedback=False):
        ''' V-Sido CONNECTに「IK設定」コマンドの送信 '''
        if not isinstance(ik_data_set, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        for ik_data in ik_data_set:
            if 'kid' in ik_data:
                if isinstance(ik_data['kid'], int):
                    if ik_data['kid'] < 0 or ik_data['kid'] > 15:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
            if 'kdt' in ik_data:
                if isinstance(ik_data['kdt'], dict):
                    if 'x' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['x'], int):
                            if ik_data['kdt']['x'] < -180 or ik_data['kdt']['x'] > 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                                return
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                            return
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                    if 'y' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['y'], int):
                            if ik_data['kdt']['y'] < -180 or ik_data['kdt']['y'] > 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                                return
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                            return
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
                    if 'z' in ik_data['kdt']:
                        if isinstance(ik_data['kdt']['z'], int):
                            if ik_data['kdt']['z'] < -180 or ik_data['kdt']['z'] > 180:
                                raise ConnectParameterError(sys._getframe().f_code.co_name)
                                return
                        else:
                            raise ConnectParameterError(sys._getframe().f_code.co_name)
                            return
                    else:
                        raise ConnectParameterError(sys._getframe().f_code.co_name)
                        return
                else:
                    raise ConnectParameterError(sys._getframe().f_code.co_name)
                    return
            else:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        if not isinstance(feedback, bool):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
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

    def _parse_ik_response(self, response_data):
        ''' 「IK設定」のレスポンスデータのパース '''
        if not isinstance(response_data, list):
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if len(response_data) < 9:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if not response_data[1] == Connect._COMMAND_OP_IK:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
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
        ''' V-Sido CONNECTに「移動情報指定（歩行）」コマンドの送信 '''
        if isinstance(forward, int):
            if forward < -100 or forward > 100:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
        if isinstance(turn_cw, int):
            if turn_cw < -100 or turn_cw > 100:
                raise ConnectParameterError(sys._getframe().f_code.co_name)
                return
        else:
            raise ConnectParameterError(sys._getframe().f_code.co_name)
            return
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

    def _send_data(self, _COMMAND_data):
        ''' V-Sido CONNECTにシリアル経由でデータ送信 '''
        if not self._connected:
            raise ConnectNotConnectedError(sys._getframe().f_code.co_name)
            return
        data_bytes = b''
        for data in _COMMAND_data:
            data_bytes += data.to_bytes(1, byteorder='little')
        self._serial.write(data_bytes)
        self._post_send_process(_COMMAND_data)

    def _send_data_wait_response(self, _COMMAND_data):
        ''' V-Sido CONNECTにシリアル経由でデータ送信して受信を待つ '''
        self._response_waiting_buffer = []
        try:
            self._send_data(_COMMAND_data)
        except ConnectNotConnectedError:
            raise
            return
        while not self._response_waiting_buffer:
            pass
        return self._response_waiting_buffer

    def _make_2byte_data(self, value):
        ''' 2Byteデータの処理 (「V-Sido CONNECT RC Command Reference」参照) '''
        value_bytes = value.to_bytes(2, byteorder='big', signed=True)
        return [(value_bytes[1] << 1) & 0x00ff, (value_bytes[0] << 2) & 0x00ff]

    def _adjust_ln_sum(self, _COMMAND_data):
        ''' データ中のLN(Length)とSUM(CheckSum)の調整 '''
        ln_pos = 1 if _COMMAND_data[0] == 0x0c or _COMMAND_data[0] == 0x0d or _COMMAND_data[0] == 0x53 or _COMMAND_data[0] == 0x54 else 2
        if len(_COMMAND_data) > 3:
            _COMMAND_data[ln_pos] = len(_COMMAND_data);
            sum = 0;
            for data in _COMMAND_data:
                sum ^= data
            _COMMAND_data[len(_COMMAND_data) - 1] = sum
            return _COMMAND_data


class ConnectParameterError(Exception):
    ''' V-Sido CONNECT Command Parameter Error '''
    pass


class ConnectNotConnectedError(Exception):
    ''' V-Sido CONNECT Not Connected Error '''
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
    while True:
        time.sleep(1)
