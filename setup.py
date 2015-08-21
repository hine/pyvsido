from distutils.core import setup
setup(
    name = 'vsido',
    packages = ['vsido'],
    version = '0.0.9',
    description = 'V-Sido Python Library',
    author = 'Daisuke IMAI',
    author_email = 'hine.gdw@gmail.com',
    url = 'https://github.com/hine/PythonVSido_Library',
    download_url = 'https://github.com/hine/PythonVSido_Library/archive/master.zip',
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
