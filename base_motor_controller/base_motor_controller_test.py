from motorsync import MotorSync
import utime

motosync = MotorSync(20, 19, 18, 11, 12, 13, 10, 16)
cnt = 0

# while True:
#     cnt += 1
#     print(" ----- Run: " + str(cnt) + " ------")
motosync.run(90)
utime.sleep_ms(5000)
motosync.stop()
#     motosync.run(15)
#     utime.sleep_ms(1000)
#     motosync.stop()
#     utime.sleep_ms(1000)
#     cnt += 1
#     print(" ----- Run: " + str(cnt) + " ------")
#     motosync.run(-70)
#     utime.sleep_ms(1000)
#     motosync.stop()
#     utime.sleep_ms(1000)
motosync.turn(-20)
utime.sleep_ms(10000)
motosync.stop()
