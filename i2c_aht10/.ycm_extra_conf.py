flags = [
        '-Wall',
        '-Wextra',
        '-Wno-variadic-macros',
        '-fexceptions',
        '-DNDEBUG',
        '-DUNIT_TESTS',
        '-std=c++17',
        '-x', 'c++',
        '-isystem', '/home/pico/pico-sdk',
        '-isystem', '/usr/lib/gcc/x86_64-linux-gnu/4.8/include',
        '-isystem', '/usr/bin/arm-none-eabi-c++',
        '-I', '~/pico/pico-sdk/',
        '-isystem', '/usr/include',
        '-isystem', '/usr/local/include',
        ]
