# This python 3 script creates a text file 
# "timestamps.txt" with timestamps for image
# data that is not time synched, such as from
# a screen recording of a video game.

# This will overwrite any existing file.
file = open("timestamps.txt", "w")

# Write the timestamps in format "XXXXX" per line.
for i in range(21, 1801+10, 10):
    # Pad nums shorter than 5 digits with 0s.
    file.write(str(i).zfill(5)+"\n")
