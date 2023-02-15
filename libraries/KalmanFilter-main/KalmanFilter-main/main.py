from controller.strapdown import StrapDown
from controller.bn220 import BN220
import time
def main():
    nav = StrapDown()
    return
    gps = BN220()
    while True:
        lat,lon,alt = gps.get_pose()

if __name__ == "__main__":
    main()