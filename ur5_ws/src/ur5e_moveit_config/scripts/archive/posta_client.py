from operator import truediv
import socket
import time

HOST = "143.27.18.83" # Posta Server Desktop PC
PORT = 1978

class PostaClient:
    
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send_msg_and_receive(self, msg, maxWaitTimeMsec = 500):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = self.socket.connect((HOST,PORT))
        self.socket.send(msg)
        data = self.socket.recv(1024)
        self.socket.close()
        return data

    def PingServer(self):
        msg = b"Ping<EOF>"
        response = self.send_msg_and_receive(msg)
        if (response is not None):
            return True
        else:
            return False

    def StartStrength(self):
        msg = b"StartStrength<EOF>"
        response = self.send_msg_and_receive(msg)
        if response is not None:
            if 'OK' in response:
                return True
            else:
                return False
        else:
            return False

    def IsInstrumentReady(self):
        msg = b"IsInstrumentReady<EOF>"
        response = self.send_msg_and_receive(msg)
        if response is not None:
            if "TRUE" in response:
                return True
            else:
                return False
        else:
            return False

    def IsTestComplete(self):
        msg = b"IsTestComplete<EOF>"
        response = self.send_msg_and_receive(msg)
        if response is not None:
            if "TRUE" in response:
                return True
            else:
                return False
        else:
            return False

    def GetPlateDistance(self):
        msg = b"GetDistanceValue<EOF>"
        response = self.send_msg_and_receive(msg)
        if response is not None:
            if "<EOF>" in response:
                s = response.replace('<EOF>','')
                try:
                    v = float(s)
                    return v
                except BaseException as e:
                    print('Error while converting {} to test: {}'.format(s,e))
                    return None
        else:
            return None

    def IsPlateMovingUp(self):
        msg = b"PlateGoingUp<EOF>"
        response = self.send_msg_and_receive(msg)
        if response is not None:
            if "<EOF>" in response:
                s = response.replace('<EOF>','')
                if ('True' in s) or ('TRUE' in s):
                    return True
                else:
                    return False
        else:
            return None


# Ping the server
sck = PostaClient()
print('Pinging Posta Server:')
if sck.PingServer():
    print('   --> Posta Server ONLINE')
    
    print('Plate distance')
    print(sck.GetPlateDistance())

    if sck.IsInstrumentReady():
        print('   --> Instrument is READY')

        # start the test
        print('Starting Strength Test!!! Time: {}'.format(time.ctime(time.time())))        
        sck.StartStrength()
        time.sleep(2) # allow a minimum time for the test to start

        test_completed = False
        while not test_completed:
            time.sleep(1)
            test_completed = sck.IsTestComplete()
            plate_distance = sck.GetPlateDistance()
            plate_going_up = sck.IsPlateMovingUp()
            print('---> At {} testComplete = {} Plate={}mm GoingUp={}'.format(time.ctime(time.time()),test_completed,plate_distance, plate_going_up))
        
        print ('PROCEDURE COMPLETED!!!')

    else:
        print('   --> Instrument NOT READY')
    
else:
    print('   --> Posta Server OFFLINE')
