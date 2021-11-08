import os
a = 10
# while agent < 40:
# ex = 0
# j = 2
while a <= 10 :
	seed = 0
	while seed<5:
		iter = 0
		while iter<2:
			f = open("sample-param.txt","r")
			filestring = ""
			for s in f:
				temp = s
				s1 = str.split(s,"=")
				#print(s1)
				if s1[0] == "agentnum":
					string = s1[0] + "=" + str(a) + "\n"
					filestring += string
				elif s1[0] == "iter":
					string = s1[0] + "=" + str(iter) + "\n"
					filestring += string
				# elif s1[0] == "field":
				# 	string = s1[0] + "=./map/32by32_agents10" + "_ex" + str(ex)+ ".map\n"
				# 	filestring += string
				elif s1[0] == "seed":
					string = s1[0] + "=" + str(seed) + "\n"
					filestring += string
				else:
					filestring += temp 
			print(filestring)
			f.close()
			fw = open("sample-param.txt","w") 
			fw.write(filestring)
			fw.close()
			#print(!make crun param=sample-param.txt)
			os.system("make crun param=sample-param.txt")
			iter += 1
		seed += 1
		
	a += 5
		# agent += 10


# seed = 0
# while seed < 100:
# 	j = 1.2
# 	while j<=2:
# 		#a = 18
# 	# while a<=20:
# 		f = open("sample-param.txt","r")
# 		filestring = ""
# 		for s in f:
# 			temp = s
# 			s1 = str.split(s,"=")
# 			# print(s1)
# 			# if s1[0] == "agentnum":
# 			# 	string = s1[0] + "=" + str(a) + "\n"
# 			# 	#print("Hello")
# 			# 	filestring += string 
# 			if s1[0] == "suboptimal":
# 				string = s1[0] + "=" + str(j) + "\n"
# 				filestring += string
# 			elif s1[0] == "seed":
# 				string = s1[0] + "=" + str(seed) + "\n"
# 				filestring += string
# 			else:
# 				filestring += temp 
# 		print(filestring)
# 		f.close()
# 		fw = open("sample-param.txt","w") 
# 		fw.write(filestring)
# 		fw.close()
# 		#print(!make crun param=sample-param.txt)
# 		os.system("make crun param=sample-param.txt")
# 		# a += 5
# 		j += 0.2
# 	seed += 1
		
	
