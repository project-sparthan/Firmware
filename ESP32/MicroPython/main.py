from network import mDNS, ftp, telnet, AP_IF, STA_IF, WLAN
from machine import idle

mdns = mDNS()
ap = WLAN(AP_IF)
ap.active(True)
ap.config(essid='Sparthan')
ap.config(authmode=3, password='123456789')

print('WLAN connection succeeded!')
mdns.start('sparthan', 'MicroPython Sparthan ESP32')
ftp.start(user='sparthan', password='123456789')
#telnet.start(user='sparthan', password='123456789')
