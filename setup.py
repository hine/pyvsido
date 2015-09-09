from distutils.core import setup
setup(
    name = 'pyvsido',
    packages = ['vsido'],
    version = '0.1.1',
    description = 'V-Sido CONNECT Livrary for Python3',
    author = 'Daisuke IMAI',
    author_email = 'hine.gdw@gmail.com',
    url = 'https://github.com/hine/pyvsido',
    download_url = 'https://github.com/hine/pyvsido/archive/master.zip',
    keywords = ['robot', 'robotics', 'V-Sido', 'serial'],
    install_requires=[
        'pyserial==2.7',
    ],
    classifiers = [
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Development Status :: 4 - Beta',
        'Natural Language :: Japanese',
        'Intended Audience :: Developers',
        'Operating System :: MacOS',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'License :: OSI Approved :: MIT License',
        ],
    long_description = '''
    '''
)
