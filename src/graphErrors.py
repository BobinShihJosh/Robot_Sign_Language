import matplotlib.pyplot as plt

# 0: no noise, 1: ..
bitError =[0,0,0,1,1,1,0,1,2,2, 1, 3, 7, 8,9,12,14, 15,15,34,61,70,85,99,100]
Noise = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,22,23, 24]
						#,0.00000010,.00001,0.00005,0.0001,0.0005,0.001,0.005,0.01]


plt.axhline(y=00, linestyle=':',linewidth = 1.5, color='0.3')
plt.axhline(y=20, linestyle=':',linewidth = 1.5, color='0.3')
plt.axhline(y=40, linestyle=':',linewidth = 1.5, color='0.3')
plt.axhline(y=60, linestyle=':',linewidth = 1.5, color='0.3')
plt.axhline(y=80, linestyle=':',linewidth = 1.5, color='0.3')

plt.plot(Noise, bitError, label='Bit Error',linewidth='2', color='r')
plt.xlabel('Noise Percentage [%]')
plt.ylabel('Bit Error Percentage [%]')
plt.title('Bit Error vs Noise')
plt.legend()
plt.show()

# plt.axhline(y=00, linestyle=':',linewidth = 1.5, color='0.3')
# plt.axhline(y=20, linestyle=':',linewidth = 1.5, color='0.3')
# plt.axhline(y=40, linestyle=':',linewidth = 1.5, color='0.3')
# plt.axhline(y=60, linestyle=':',linewidth = 1.5, color='0.3')
# plt.axhline(y=80, linestyle=':',linewidth = 1.5, color='0.3')
# plt.plot(snr, SpeedError, label='Speed Error',linewidth=2, color ='b')
# plt.xlabel('Signal to Noise Ratio (SNR) [dB]')
# plt.ylabel('Speed Error Percentage [%]')
# plt.title('Speed Error vs Signal Noise Level')
# plt.legend()
# plt.show() 