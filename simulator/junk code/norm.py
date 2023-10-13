import sys

def norm(x1,y1,x2,y2):
	d1=(x1-x2)**2
	d2=(y1-y2)**2
	norm=(d1+d2)**0.5
	return norm

if len(sys.argv) < 4:
	print("USAGE : INSERT x_1 y_1 x_2 y_2 in this specific order \n")
	quit()
else:
	print("value of the norm is : " +  str(norm(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]))))
	
