 ###########################################################################
# 1) connect to internet via SPI_2_Eth
# 2) get time
# 3) report data to RPi MQTT



# pinout
# D1 - PIR Motion Sensor
# D2 - Digital Humidity Temperature
# D9 - Sound Detector D0
# D10 - Gas Detector D0
# A0 (ADC) - Gas Detector A0
##########################################################################

import machine
import utime
import ntptime
import network
import esp
from dht import DHT11
import ubinascii
from umqtt.robust import MQTTClient
DEBUG = True

UTC_OFFSET = 3   # hours of differenc between UTC and local (Jerusalem) time
COMMANDS_PORT = 5641  # port at RPi to send commands
RPi_HOST = "10.0.0.17"


CARBON_MONOXIDE_ADC_THRESH = 300


#Logging constants
LOG_DELAY = 5*60    # minimal number of seconds to wait/
                            # between subsequent sensor Logging
SAMPLE_DELAY = 2
MOTION_LOG_DELAY = 5*60   # minimal number of seconds to wait/
                            # between subsequent motion Logging


#Define MQTT topics:
TEMPERATURE_CHIPA = "/sensor/Chipa/temperature"
HUMIDITY_CHIPA = "/sensor/Chipa/humidity"
MOVEMENT_CHIPA = "/sensor/Chipa/motion"
CARBON_MONOXIDE_CHIPA = "/sensor/Chipa/CO"
SOUND_LEVEL_CHIPA = "/sensor/Chipa/sound"


clientID =  b"chipa_ESP8266"
localBroker = RPi_HOST


#global variable, change only in the appropriate function
lastCOLogTime = 0
motionDetected = False
lastMotionTime = 0
lastMotionLogTime = 0
lastTemperatureLogTime = 0


def toggleGPIO(p):
    p.value(not p.value())


def GPIO_On(p):
    p.value(1)


def GPIO_Off(p):
    p.value(0)


def getHumidityTemp():
    try:
        dhtSensor.measure()
        temper = dhtSensor.temperature()
        humid = dhtSensor.humidity()
    except:
        (temper, humid) = (None, None)

    return temper, humid


def getDateTime():
    gotTime = False
    utime.sleep(5)  # Sleep for 5 seconds, to make sure WiFi is connected
    if sta_if.isconnected():
        print("Connected to internet, setting time from NTP")
        ntptime.settime()
    t = rtc.datetime()
    print("RTC time is: ", t)
    year = t[0]
    print("year is: ",year)
    gotTime = year > 2016
    if gotTime:
        print(t)
    else:
        print("Could not get time")
        machine.reset()
    return gotTime, t 

        
def getRPiTime():
    import socket
    addr = socket.getaddrinfo(RPi_HOST, COMMANDS_PORT)[0][-1]
    s = socket.socket()
    print("Connecting to RPi: ", RPi_HOST)
    try:
    #TODO Something wrong here, the socket does not connect. \
    # Important, after failed connection need to open a new socket.
        s.connect(addr)
    except:
        print("Error connecting to RPi server")
        s.close()
        return None

    s.send("time".encode("utf8"))
    s.send("time".encode("utf8"))
    time.sleep(3)
    data = s.recv(1024)
    payload = data.decode("utf8")
    if data:
        print('commad received: ' + payload)
    else:
        print('No data received')
    s.close()
    return None


def pin_interrupt():
    global motionDetected, lastMotionTime
    lastMotionTime = utime.time()
    motionDetected = True


def handleDHT(temper, humid,currentTime):
    global lastTemperatureLogTime
    print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid, 
           "time[s]: ", currentTime)
    if (lastTemperatureLogTime == 0) or \
            currentTime - lastTemperatureLogTime > LOG_DELAY:
        print ("LOGGING temperature and humidity at: ", currentTime)
        lastTemperatureLogTime = currentTime
        pushSample(temper,TEMPERATURE_CHIPA)
        pushSample(humid,HUMIDITY_CHIPA)


def handlePIR(currentTime):
    global lastMotionTime, lastMotionLogTime
    print ("Movement detected at: ", lastMotionTime)
    if (lastMotionLogTime == 0) or \
            (currentTime - lastMotionLogTime) > LOG_DELAY:
        print ("LOGGING motion at: ", currentTime)
        lastMotionLogTime = currentTime
        if not DEBUG:
            pushSample(lastMotionTime, "/sensor/Chipa/motion")

def handleCO(gasA0,gasDetected,currentTime):
    global lastCOLogTime
    print("Analog gas value: ",gasA0, " digital gas threshold value: ", gasDetected)
    alert = gasA0 > CARBON_MONOXIDE_ADC_THRESH
    if alert:
        print("ALERT!!!!! CARBON MONOXIDE LEVEL ABOVE THRESHOLD")

    if alert or (lastCOLogTime == 0) or \
            (currentTime - lastCOLogTime) > LOG_DELAY:
        print ("LOGGING Carbon Monoxidde at: ", currentTime)
        lastCOLogTime = currentTime
        pushSample(gasA0,CARBON_MONOXIDE_CHIPA)


def pushSample(sample, topic):
    global client
    client.publish(topic, str(sample))


def MQTT_Connect(client):
    isMQTT_Connected = False
    while not isMQTT_Connected:
        try:
            utime.sleep(1)
            client.connect()
            print("Connected to {}".format(localBroker))
            isMQTT_Connected = True
            return
        except:
            print("failed to connect to RPi MQTT broker. waiting 3sec and retrying")
            utime.sleep(3)

def inetConnect():
    """Connect to the internet using the SPI to Eth interface chip, w5500"""
    """sck, mosi, miso are pins (machine.Pin)"""

#Generic Init
print ("Initializing...")
rtc = machine.RTC()
#sta_if = network.WLAN(network.STA_IF)
eth_if = inetConnect()


dhtSensor = DHT11(machine.Pin(4))  # D2 pin on NodeMCU board. DHT signal pin
pirSig = machine.Pin(5, machine.Pin.IN)          # D1 pin on NodeMCU. PIR signal pin
pirSig.irq(handler=lambda p: pin_interrupt(), trigger=machine.Pin.IRQ_RISING)

#Init SPI
#MISO is GPIO12(D6), MOSI is GPIO13(D7), and SCK is GPIO14(D5)
###hspi = machine.SPI(1, baudrate=40000000, polarity=0, phase=0)

#Carbon Monoxide sensor
#Init ADC
adc = machine.ADC(0)
#Threshold pin
gasD0 = machine.Pin(1)

#MQTT configs
if not DEBUG:
    client = MQTTClient(clientID, localBroker)
    MQTT_Connect(client)

while True:

    if DEBUG:
        #loop reading and printing all sensor inputs
        utime.sleep(3)  # sleep for 3 seconds
        print("reading Temp&Humid")
        temper, humid = getHumidityTemp()
        print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid)
        print("reading Gas analog and threashold")
        gasA0 = adc.read()
        gasDetected = gasD0.value() == 1
        print("Analog gas value: ",gasA0, " digital gas threshold value: ", gasDetected)
        if gasA0 > CARBON_MONOXIDE_ADC_THRESH:
            print("ALERT!!!!! CARBON MONOXIDE LEVEL ABOVE THRESHOLD")
        if motionDetected:   
            motionDetected = False
            handlePIR()

    else:
        #gotTime, curr_tm = getDateTime()  # get time 
        utime.sleep(SAMPLE_DELAY)  # Sleep at least one second between each measurement
        currentTime = utime.time()

        # sample the temperature and humidity sensor
        temper, humid = getHumidityTemp()
        handleDHT(temper, humid,currentTime)
        
        #sample carbon monoxide
        gasA0 = adc.read()
        gasDetected = gasD0.value() == 1
        handleCO(gasA0,gasDetected,currentTime)

        # if detected motion on PIR sensor
        if motionDetected:   
            motionDetected = False
            handlePIR(currentTime)
