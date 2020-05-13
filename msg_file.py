import rosbag
from geometry_msgs.msg import PoseStamped

'''This program takes in a user input of a rosbag file containing messages from a robot navigation and reduces the number of messages in the rosbag file by 
extracting the message in the centre of each group in N or N+1 groups. A reduced number of messages in a given frequency reduces the number of waypoints
for the robot to follow while preserving the robot path.''' 

N = input("Enter no. waypoints:") 
inbag = input("Enter inbag file name:", ) 
outbag = input("Enter outbag file name:")


'''function groups a list of items with n items  in each group, and the remainders in a seperate group at the end of the list'''
def out_lst(arr,n):
	out_var = []
	grpd_lst = [arr[k:k+n] for k in range(0,len(arr), n)]

	# center element(if odd) or the right of the 2 center elements(if even) is identified and added  into a out_var list
	for i in range(len(grpd_lst)):
		out_var.append(grpd_lst[i][len(grpd_lst[i])//2])
	return (out_var)



'''function takes in the inbag, outbag and N value given by user input, reads the messages in the inbag and groups the messages using the out_lst method'''
def output(inbag,outbag,N):
	topic_in = []
	msg_in = []
	t_in = []

	for topic, msg, t in rosbag.Bag(inbag).read_messages():
		topic_in.append(topic)
		msg_in.append(msg)
		t_in.append(t)

	msg_length = len(topic_in)
	n = msg_length//N

	out_topic = out_lst(topic_in,n)
	out_msg = out_lst(msg_in,n)
	out_t = out_lst(t_in,n)

	#once the center message of each group is obtained and stored in a list format, it is extracted and from a for loop iteration and written in a outbag file.
	with rosbag.Bag('outbag.bag', 'w') as outbag:
		for topic, msg, t in zip(out_topic, out_msg, out_t):
			outbag.write(topic, msg, t)

output(inbag,outbag, N)



