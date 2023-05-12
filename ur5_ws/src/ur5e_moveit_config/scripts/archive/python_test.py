import time

yy = time.strftime("%y", time.localtime())
mm = time.strftime("%m", time.localtime())
dd = time.strftime("%d", time.localtime())
cnt = 13
print('SUD_{:s}{:s}{:s}_{:03d}'.format(yy,mm,dd,cnt))


