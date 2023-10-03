read = open('keybinds.txt','r')
lines = read.readlines()
for line in lines:
    if 'test' in line:
        print('yes')
    else:
        print('no')

read.close()