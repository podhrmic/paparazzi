  #!/usr/bin/env python

'''
Generate a random public-private keypairs
 - one keypair for Ground Control Station
 - pne keypair for UAV

Copyright (C) 2017 David Cerny <david.cerny@senman.cz>
For the Paparazzi UAV and PPRZLINK projects

    Released under GNU GPL version 3 or later


'''
import sys, textwrap, os
generatedFilenames = ["keys_uav.h", "keys_gcs.h"]

if __name__ == "__main__":
    from argparse import ArgumentParser
    import os.path
    import random
    from random import SystemRandom
    import jinja2

    parser = ArgumentParser(description="This tool generate keys_uav/keys_gcs with random bytes.")
    parser.add_argument("output_directory",  default='./output/', help="output path: i.e. ./output/") # ending slash is important
    args = parser.parse_args()
    cryptogen = SystemRandom()
    r = lambda: ', '.join('0x{0:X}'.format(k) for k in [random.randint(0, 255) for _ in xrange(32)])

    data={}
    for fileName in generatedFilenames:
        field_array = []

        field = {}
        field['name'] = 'private_key'
        field['length'] = 32
        field['bytes'] = r()
        field_array.append(field)

        field = {}
        field['name'] = 'public_key'
        field['length'] = 32
        field['bytes'] = r()
        field_array.append(field)

        data[fileName] = field_array

    templateLoader = jinja2.FileSystemLoader(searchpath="./")  ## search local folder
    templateEnv = jinja2.Environment(loader=templateLoader)

    for fileName in generatedFilenames:
        print 'Generating:', fileName
        template = templateEnv.get_template(fileName + '_template')
        outputText = template.render(keys=data[fileName])
        directory = os.path.join(args.output_directory, '')

        if not os.path.exists(directory):
            os.makedirs(directory)

        file = open(os.path.join(directory, fileName), 'w')
        file.write(outputText)
        file.close()
