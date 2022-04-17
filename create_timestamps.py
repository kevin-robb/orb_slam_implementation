# This python 3 script creates a text file 
# "timestamps.txt" with timestamps for our
# image data that follows the naming scheme
# "frameXXXX.jpg", where XXXX is the 4 digit
# frame number.

number_of_images = 5244

# This will overwrite any existing file.
file = open("timestamps.txt", "w")

# Write the timestamps in format
# XXXX,frameXXXX.jpg per line.
for i in range(1,number_of_images+1):
    # Pad nums shorter than 4 digits with 0s.
    file.write(str(i).zfill(4)+",frame"+str(i).zfill(4)+".jpg\n")
