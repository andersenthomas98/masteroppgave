
import json

point_file = open('point_log.txt', 'r')

c_code = open('code3.txt', 'w')

i = 0 
prev_id = 0
for entry in point_file.readlines():
    log = json.loads(entry)
    if (log['id'] != prev_id):
        c_code.write('point_buffers[{}].len = {};\n'.format(prev_id ,i))
        prev_id = log['id']
        i = 0
    c_code.write('point_buffers[{}].buffer[{}] = (point_t){{.x = {}, .y = {}, .label = LABEL_UNDEFINED}}; \n'.format(log['id'], i, log['x'], log['y']))
    i += 1
    
point_file.close()
c_code.close()