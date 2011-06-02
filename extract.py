import re

# this could probably be hacked together from grep and sed, but as we're using python for other things
# we can assume that we have a python installation available.

# (additionally we could generate markers to inject changes made on the Wiki back into the source file)

re_comment = re.compile(r"\s*//\?\s?(.*)")

f = open("gcode_process.c", "rt")
doc = open("gcode_doc.txt", "wt")
for line in f.readlines():
    m = re_comment.match(line)
    if m:
	doc.write(m.group(1) + "\n")
f.close()
doc.close()
