import json
from collections import OrderedDict

status = {}
status['drawer_1'] = 'closed'
status['drawer_2'] = 'closed'
status['drawer_3'] = 'closed'
status['drawer_4'] = 'open'
status['drawer_5'] = 'closed'
status['drawer_6'] = 'closed'

for i in range(6):
    drawer_name = 'drawer_{0}'.format(i+1)
    if(drawer_name in status):
        if(status[drawer_name]=='open'):
            print('Drawer {0} is OPEN!'.format(i+1))
        else:
            print('Drawer {0} is closed'.format(i+1))
