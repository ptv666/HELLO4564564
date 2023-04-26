import numpy as np  
import matplotlib.pyplot as plt 
from matplotlib import style
from mpl_toolkits.mplot3d import axes3d


#六面体的顶点
point1= np.array([2.60003, -2.61483, 13.8055,
5.41099, 16.1854, 14.4224,
1.9651, -7.5831, -4.04642,
4.77605, 11.2171, -3.42953,
-4.77605, -11.2171, 3.42953,
-1.9651, 7.5831, 4.04642,
-5.41099, -16.1854, -14.4224,
-2.60003, 2.61483, -13.8055])

x1 = point1[0::3]
y1 = point1[1::3]
z1 = point1[2::3]

#交点集合
point2= np.array([-2.09181,-2.09181,5.91654,
-1.90047,1.90047,5.37533,
3.54537,3.54537,10.0278,
2.63566,-2.63566,7.45476,
3.73394,4.96895,14.0543,
1.23782,-4.20349,11.8893,
2.47247,-3.61296,10.219,
0,0,0,
2.600036680, 2.614822210, 13.80550940])
x2 = point2[0::3]
y2 = point2[1::3]
z2 = point2[2::3]


#三维绘图
style.use('ggplot')
fig = plt.figure()
ax1 = fig.add_subplot(111, projection='3d')

ax1.scatter(x2, y2, z2, c='r', marker='o')	#交点

#------六面体绘制：点线法------
# ax1.scatter(x1, y1, z1, c='g', marker='o')	#六面体的顶点
list_str = ['-7','-5','-3','-1','1','3','5','7']	#标记hex的顶点
for i in range(8):
	ax1.text(x1[i],y1[i],z1[i],list_str[i])
#绘制六面体的边线：
hex_map = {	-7:0,
						-5:1,
						-3:2,
						-1:3,
						1:4,
						3:5,
						5:6,
						7:7}
hex_line = np.array([ -1, -5,
		-1, 7,
		-1, -3,
		-5, 3,
		-5, -7,
		3, 1,
		3, 7,
		1, -7,
		1, 5,
		5, 7,
		5, -3,
		-3, -7]);
for i in range(12):
	x_temp = [x1[hex_map[hex_line[2*i]]],x1[hex_map[hex_line[2*i + 1]]]]
	y_temp = [y1[hex_map[hex_line[2*i]]],y1[hex_map[hex_line[2*i + 1]]]]
	z_temp = [z1[hex_map[hex_line[2*i]]],z1[hex_map[hex_line[2*i + 1]]]]
	ax1.plot(x_temp,y_temp,z_temp, c='y')


#------绘制线性摩擦锥-------
k = 2.828424428
#绘制四棱锥：点线法
point3 = np.array([0,0,0,	5,5,5*k,	5,-5,5*k,	-5,-5,5*k,	-5,5,5*k])
print(point3)
point3 = point3*1.2;
x3 = point3[0::3]
y3 = point3[1::3]
z3 = point3[2::3]
cone_line = np.array([0,1,	0,2,	0,3,	0,4,	1,2,	2,3,	3,4,	4,1])
for i in range(8):
	x_temp = [x3[cone_line[2*i]],x3[cone_line[2*i + 1]]]
	y_temp = [y3[cone_line[2*i]],y3[cone_line[2*i + 1]]]
	z_temp = [z3[cone_line[2*i]],z3[cone_line[2*i + 1]]]
	ax1.plot(x_temp,y_temp,z_temp, c='g')





#------交集凸包的绘制------
list_str = ['1','2','3','4','5','6','7','8','9']
for i in range(9):
	ax1.text(x2[i],y2[i],z2[i],list_str[i])
cross_line = np.array([1,2,	2,5,	5,3,	3,4,	4,7,	7,6,	6,1,	1,4,	2,3,	5,6,	1,8,	2,8,	3,8,	4,8])
cross_line =cross_line - 1
#连接交点构成凸包
for i in range(int(len(cross_line)/2)):
	x_temp = [x2[cross_line[2*i]],x2[cross_line[2*i + 1]]]
	y_temp = [y2[cross_line[2*i]],y2[cross_line[2*i + 1]]]
	z_temp = [z2[cross_line[2*i]],z2[cross_line[2*i + 1]]]
	ax1.plot(x_temp,y_temp,z_temp, c='b')



ax1.set_title("Feasible Force Polytope")
ax1.set_xlabel('x axis')
ax1.set_ylabel('y axis')
ax1.set_zlabel('z axis')
plt.axis('equal')
plt.show()