import os
import sys


def ls():
    print(os.listdir())


def cat(filename=""):
    if filename == "":
        print("filename is a required parameter")
    else:
        try:
            f = open(filename, 'r')
        except:
            print("Error opening file:" + filename)
        else:
            file_contents = f.read()
            print(file_contents)
            f.close()
