import serial

degrau = False
nsenos = True

if degrau:
    TEMPO_TOTAL = 15
    Ts = 0.01

if nsenos:
    TEMPO_TOTAL = 60
    Ts = 0.02

arduino_port = "/dev/ttyUSB0"
baud = 1000000
filelocation = "../pos processamento/data/raw/"
filename = "data.csv"
samples = 10000

ser = serial.Serial(arduino_port,baud)
print("Connected to arduino port: " + arduino_port)
file = open(filelocation+filename,"w")
file.close()

print("Created file")

line = 0
tempo = 0

while line < TEMPO_TOTAL/Ts:
    getData = ser.read(4)
    try:
        print(f"{getData[2]}, {getData[1]}, {getData[0]}, {getData[3]}")
        
        file = open(filelocation+filename,"a")

        file.write(f"{getData[2]},{getData[1]},{getData[0]},{getData[3]}\n")
        line+=1
    except Exception as e:
        print("HAHA")

file.close()
ser.close()
print("Data collection complete!")