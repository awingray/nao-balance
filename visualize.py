import matplotlib.pyplot as plt
import pickle

def main():
	with open('data.pkl', 'rb') as f:
		data = pickle.load(f)
	with open('data_balance.pkl', 'rb') as f:
		data_balance = pickle.load(f)

	length = len(data['time']) - 50
	plt.plot(data['time'][:length], data['xCom'][:length][:length], label="xCoM")
	plt.plot(data['time'][:length], data['b1_custom'][:length], label="b1_custom")
	# plt.plot(data['time'][:length], data['b2_custom'][:length], label="b2_custom")
	# plt.plot(data['time'][:length], data['b1'][:length], label="b1")
	# plt.plot(data['time'][:length], data['b2'][:length], label="b2")
	plt.plot(data['time'][:length], data['acc'][:length], label="acc")
	plt.title("Without balance controller")
	plt.ylabel('CoM on x-axis (cm)')
	plt.xlabel('Time (secs)')
	plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5, mode="expand", borderaxespad=0.)
	plt.xlim(0.5, 4)
	plt.show()

	length = len(data_balance['time']) - 50
	plt.plot(data_balance['time'][:length], data_balance['xCom'][:length], label="xCoM Bal")
	plt.plot(data_balance['time'][:length], data_balance['b1_custom'][:length], label="b1_custom")
	# plt.plot(data_balance['time'][:length], data_balance['b2_custom'][:length], label="b2_custom")
	# plt.plot(data_balance['time'][:length], data_balance['b1'][:length], label="b1")
	# plt.plot(data_balance['time'][:length], data_balance['b2'][:length], label="b2")
	plt.plot(data_balance['time'][:length], data_balance['acc'][:length], label="acc")
	plt.title("With balance controller")
	plt.ylabel('CoM on x-axis (cm)')
	plt.xlabel('Time (secs)')
	plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5, mode="expand", borderaxespad=0.)
	plt.xlim(0.5, 4)
	plt.show()
		
	
if __name__ == "__main__":
	main()

