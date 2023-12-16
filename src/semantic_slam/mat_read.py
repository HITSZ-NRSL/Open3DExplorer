#coding:UTF-8
'''
Created on 2015年5月12日
@author: zhaozhiyong
'''

'''
脚本是改变各个label的颜色显示，通过查表查每行对应的label
'''

import scipy.io as scio
 
dataFile = 'color150.mat'
data = scio.loadmat(dataFile)

# print data.keys()
# print data["colors"][0]

# water_list = [3, 9, 13, 22, 27, 28, 61, 110, 114, 127, 129, 148]
water_list = [2, 8, 12, 21, 26, 27, 60, 109, 113, 126, 128, 147] # PSPNET表格对应的行号，2对应第三行
length = len(data["colors"])
print data["colors"][length-1]
print length
for i in range(length):
    if i in water_list:
        data["colors"][i] = [61, 230, 250]  # 将相应行颜色标为[61, 230, 250]
    else: 
        data["colors"][i] = [4, 250, 7]

# print data["colors"][0]
# print data["colors"][length-1]
# data

dataNew = 'color150_binary.mat'
scio.savemat(dataNew, {'colors':data['colors']})