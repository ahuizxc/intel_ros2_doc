
import pdb
from urllib2 import urlopen
import os
os.system('sudo apt-get install -y python3-pip python-pip')
import yaml
ubuntu_version = 'bionic'
os.system('sudo rm /tmp/base.yaml')
os.system('sudo rm /tmp/python.yaml')
os.system('sudo apt-get install wget')
os.system('wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml -P /tmp/')
os.system('wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml -P /tmp/')
print "Opening base.yaml"
f = open('/tmp/base.yaml')
print "Done!"
skip_packages = ['texmaker','texlive-latex-recommended','texlive-latex-extra','texlive-latex-base','texlive-full','texlive-latex-full','texlive-fonts-recommended','texlive-fonts-extra','libphonon4', 'libphonon-dev', 'ace','cpuburn', 'libconsole-bridge-dev', 'console_bridge', 'fastcdr fastrtps', 'libopensplice67', 'rti-connext-dds-5.3.1', 'urdfdom_headers']
f_shell = open('/tmp/base.sh','wb')
x = yaml.load(f)
apt_ = 'sudo apt-get install -y --no-install-recommends'
apt_context = ""
for key, value in x.items():
    if key in skip_packages:
        continue
    for key2, value2 in value.items():
        if key2 != 'ubuntu':
            continue
        if isinstance(value2, dict):
            try:
                #apt_context = apt_context + apt_ + ' ' + value2[ubuntu_version][0]+'\n'
                os.system(apt_+ ' '+ value2[ubuntu_version][0])
		print(value2[ubuntu_version][0])
            except:
                pass
        elif len(value2) != 0 :
	    os.system(apt_+ ' '+ value2[0])
            #apt_context = apt_context +apt_+ " " + value2[0]+'\n'
print "Opening python.yaml"   
'''f2 = open("/tmp/python.yaml")
print "Done!"
x = yaml.load(f2)
pip_context = ''
for key, value in x.items():
    if key in skip_packages:
        continue
    try:
        for key2, value2 in value.items():
            if key2 != 'ubuntu':
                continue
            if isinstance(value2, dict):
                if value2.keys() == ['pip']:
                    if isinstance(value2['pip'], dict):                   
                        #pip_context = pip_context + "pip install " + value2['pip']['packages'][0]+' -i https://pypi.douban.com/simples\n'
			os.system("pip install " + value2['pip']['packages'][0]+' -i https://pypi.douban.com/simples')
                    else:
			os.system("pip install " + value2['pip'][0]+' -i https://pypi.douban.com/simples')
                        #pip_context = pip_context + "pip install " + value2['pip'][0]+' -i https://pypi.douban.com/simples\n'
                elif isinstance(value2, dict):
                    if 'xenial' in value2.keys():
                        try:
                            #apt_context = apt_context + apt_+ " " + value2[ubuntu_version][0]+'\n'
 	                    os.system(apt_+ ' '+ value2[ubuntu_version][0])
		            #print(value2[ubuntu_version][0])
                        except:
                            if 'pip' in value2[ubuntu_version].keys():
                                if 'packages' in value2[ubuntu_version]['pip'].keys():
                                    #pip_context = pip_context + 'pip install ' + value2[ubuntu_version]['pip']['packages'][0]+' -i https://pypi.douban.com/simples\n'
			            os.system("pip install " + value2[ubuntu_version]['pip']['packages'][0]+' -i https://pypi.douban.com/simples')
                                else:
                                    #pip_context = pip_context + 'pip install ' + value2[ubuntu_version]['pip'][0]+' -i https://pypi.douban.com/simples\n'
			            os.system("pip install " + value2[ubuntu_version]['pip'][0]+' -i https://pypi.douban.com/simples')
                              
                           
                elif len(value2) != 0 :
	            os.system(apt_ + ' ' + value2[0])
                    #apt_context = apt_context + " " + value2[0] 
    except:
        pass
                
                
os.system('rm /tmp/python.yaml\n')'''
os.system('rm /tmp/base.yaml\n')
