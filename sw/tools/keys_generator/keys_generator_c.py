#!/usr/bin/env python

'''
parse a PPRZ protocol XML file and generate appropriate implementation

Copyright (C) 2017 David Cerny <david.cerny@senman.cz>
For the Paparazzi UAV and PPRZLINK projects

    Released under GNU GPL version 3 or later


'''
import sys, textwrap, os

# keys_uav.h
#
# ve kterem bude
#
# uint8_t private_key[32] = { nahodne vygenerovanych 32 bytu }
# uint8_t public_key[32] = { nahodne vygenerovanych 32 bytu }
#
# a keys_gcs.h
#
# ve kterem bude:
#
# uint8_t private_key[32] = { nahodne vygenerovanych 32 bytu }
# uint8_t public_key[32] = { nahodne vygenerovanych 32 bytu }
#
# Chtel bych specifikovat to kde se ten soubor vygeneruje jako command line argument pro ten python program, napr:
# generate_keys.py ../../var/keys

generatedFilenames = ["keys_uav.h", "keys_gcs.h"]



if __name__ == "__main__":
    from argparse import ArgumentParser
    import os.path
    import random
    import jinja2

    parser = ArgumentParser(description="This tool generate keys_uav with random bytes.")
    parser.add_argument("output_directory",  default='./output/', help="output path: i.e. ./output/") # ending slash is important
    args = parser.parse_args()

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



    # uint8_t private_key[32] = { nahodne vygenerovanych 32 bytu }
         # uint8_t public_key[32] = { nahodne vygenerovanych 32 bytu }
    #
    #
    #
    #     file.write('whatever')
    #     file.close()
    #
    # # gen_messages(args)
