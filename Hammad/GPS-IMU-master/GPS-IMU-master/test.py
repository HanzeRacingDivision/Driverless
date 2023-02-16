from imu_nmea import (t_dict)
import sys
import pandas as pd 
import os

nmea_file = []
imu_file = []

for x in os.listdir():
    if '.NMEA' in x:
        nmea_file.append(str(x))
    if '.csv' in x:
        imu_file.append(str(x))

if len(nmea_file) != len(imu_file):
    print('Неравное количество файлов .NMEA и .csv', '\n', 'Проверьте файлы в директории!')
    input('Нажмите любую клавишу для выхода')
    exit()

elif len(nmea_file) == len(imu_file) and len(nmea_file) != 0:
    for i in range(len(nmea_file)):
        print('\n', i + 1, ' пара файлов из ', len(nmea_file), '\n')
        try:        
            print('nmea_file -', nmea_file[i], '\n', 'imu_file -', imu_file[i])
        except NameError:
            x = input('В текущей директории отсутствуют нужные файлы (.NMEA, .csv)','\n',
            'Нажмите любую клавишу для выхода')
            exit()

        max_IMU_time_lag = 0.2

        while True:
            try:
                NMEA_coverage = input('Предельный интервал заполнения (default = 45sec):') #45
                if NMEA_coverage == '':
                    NMEA_coverage = float(45)
                else:
                    NMEA_coverage = float(NMEA_coverage)
            except ValueError:
                print('Некорректное значение. Введите значение в формате 66.6')
                continue
            break
        
        while True:
            try:
                NMEA_median_interval = input('Интервал усреднения последнего NMEA-угла (default = 2sec):')  #2
                if NMEA_median_interval == '':
                    NMEA_median_interval = float(2)
                else:
                    NMEA_median_interval = float(NMEA_median_interval)
            except ValueError:
                print('Некорректное значение. Введите значение в формате 66.6')
                continue
            break
        
        
        time_new = t_dict(nmea_file[i], imu_file[i], NMEA_coverage, max_IMU_time_lag, NMEA_median_interval)
        pd.set_option("display.max_rows", None, "display.max_columns", None)
        print(pd.DataFrame(time_new)) 

        if i + 1 < len(nmea_file):
            x = input('Нажмите любую клавишу для продолжения')
        else:
            x = input('Нажмите любую клавишу для завершения')
            exit()
else:
    x = input('Файлы (.NMEA, .csv) не добавлены')
    exit()

#args: (nmea_file, imu_file, near_time_NMEA, near_time_IMU, median_time):
# time_new = t_dict('test/050420/test_2/nmea_moded_for_tests.NMEA', 
                                                    # 'test/050420/test_2/imu_2.csv', 45, 0.2, 2)
# pd.set_option("display.max_rows", None, "display.max_columns", None)
# print(pd.DataFrame(time_new))    
