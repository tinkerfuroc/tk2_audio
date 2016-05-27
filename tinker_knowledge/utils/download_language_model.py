#!/usr/bin/python
import argparse
import os
import re
import requests
import rospkg
import subprocess
import xml.etree.ElementTree as ET

cmu_url = 'http://www.speech.cs.cmu.edu/cgi-bin/tools/lmtool/run'


def phrase_question(xml):
    xml_root = ET.parse(xml).getroot()
    name = xml_root.get('name')
    questions = {}
    for question in xml_root.findall('question'):
        questions[question.find('q').text] = question.find('a').text
    return questions, name


def main():
    rospack = rospkg.RosPack()
    output_path = os.path.join(rospack.get_path('tinker_knowledge'), 'model')
    parser = argparse.ArgumentParser(
        description='Download Sphinx language model form http://www.speech.cs.cmu.edu/tools/lmtool-new.html')
    parser.add_argument('-i', '--input', required=True, help="Input question xml file")
    parser.add_argument('-o', '--output', default=output_path, help="Output path")
    args = parser.parse_args()
    output_path = args.output
    question_dict, name = phrase_question(os.path.abspath(args.input))
    corpus = ''
    for question in question_dict:
        corpus += question + '\r'
    response = requests.post(cmu_url, files={'formtype': 'simple', 'corpus': corpus})
    url = response.url
    id = re.search('<b>(\d*)</b>', response.text).groups()[0]
    response = requests.get(url + id + '.dic')
    with open(os.path.join(output_path, name + '.dic'), 'wb') as out_file:
        out_file.write(response.text)
    response = requests.get(url + id + '.lm')
    with open(os.path.join(output_path, name + '.lm'), 'wb') as out_file:
        out_file.write(response.text)

    with open(os.path.join(output_path, name + '.jsgf'), 'w') as out_file:
        out_file.write('#JSGF V1.0;\n')
        out_file.write('grammar ' + name + ';\n')
        for index, question in enumerate(question_dict):
            out_file.writelines('public <{}>='.format(index) + question.upper() + ';\n')

    jsgf = os.path.join(output_path, name + '.jsgf')
    fsg = os.path.join(output_path, name + '.fsg')
    subprocess.Popen(('sphinx_jsgf2fsg', '-jsgf', jsgf, '-fsg', fsg))


if __name__ == '__main__':
    main()
