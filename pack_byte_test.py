import struct

value = -0.00022  # arbitrary float 
a= [-0.0001,-3e-05, -0.02085]

bin = struct.pack('f', 770)

print(bin, end = "\r")


# byte_input = b'\x03\x02'
# real_value = struct.unpack("H",byte_input)
# print(real_value)
