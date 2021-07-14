Introduction
============

MkECLI, or `libmkeclient`, is a C++ implementation of the client-side
MKE API v1.0 [\[mkeapi\]](#mkeapi). This document is `libmkeclient` user
guide. It is not the library’s reference manual. For a full reference,
see the [Doxygen generated source code
documentation](html/classmke_1_1cli_1_1_client.html).

The user communicates with the library mainly through the class `Client`
which resides in the `mke::cli` namespace.

Building
========

The officially supported target platform is Ubuntu 20.04 running on an
AMD64 system. However, the library can be compiled for other platforms
as well, below is an "as is" example of compilation for Windows using
MSVC compiler with CMake support.

The main dependency on `libmkeclient` is the `Boost` library. The
library depends on the header-only `mkeapi`, which provides definitions
of all MkE API related structures though the `mkeapi.h` file. Both
`mkeapi` and `libmkeclient` are part of the official MkE SDK and if
compiled from the official SDK location, the `libmkeclient` build
process is able to locate and use the `mkeapi` library automatically.

Several demo applications are built together with `libmkeclient` when
option `MKECLI_DEMO` is `ON` in `cmake` command.

Library verification tool can be built together with `libmkeclient` when
option `DMKECLI_TESTER` is `ON` in `cmake` command. More about this
testing tool can be found in its own documentation.

Building on Ubuntu
------------------

In order to compile the library on Ubuntu 20.04, the following packages
are required:

``` bash
$ sudo apt install build-essential git cmake libboost-all-dev
```

CMake is used as the build system for the library. Assuming that the
`MKECLIENT_ROOT` environment variable points to the root directory of
the `mkeclient` code base, to build the library, just execute the
following commands in BASH:

``` bash
$ mkdir ${MKECLIENT_ROOT}/build
$ cd ${MKECLIENT_ROOT}/build
$ cmake -DMKECLI_TESTER=OFF -DMKECLI_DEMO=ON ..
$ make
```

The above commands should produce results in `build/src` folder.

Library and it’s header files can then be installed to the system using:

``` bash
sudo make install
```

Library linking on Ubuntu
-------------------------

Here is an example how to link code samples from this document assuming
the `libmkeclient` is installed using above steps and the c++ code is
stored in `example.cpp` file:

``` bash
g++ -I/usr/local/include/mke/api/ -pthread example.cpp -o example -lmkeclient -lboost_system -lboost_thread
```

The above command should produce `example` binary that can be run via
`./example` command.

Building on Windows
-------------------

Download and install `Build Tools for Visual Studio 2019` from
[`microsoft.com`](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2019).

Download and install the `Boost` library in version 1.74.0 or newer,
*e.g.*, from
[`sourceforge.net`](https://vorboss.dl.sourceforge.net/project/boost/boost-binaries/).

In `x64 Native Tools Command Prompt for VS 2019` go to the
`libmkeclient` code base root folder (`mkeclient`) and execute the
following commands:

    mkdir build
    cd build
    cmake .. -A x64 -DMKECLI_TESTER=OFF -DMKECLI_DEMO=ON \
          -DBOOST_ROOT=C:\local\boost_1_74_0\boost \
          -DBOOST_INCLUDEDIR=C:\local\boost_1_74_0 \
          -DBOOST_LIBRARYDIR=C:\local\boost_1_74_0\lib64-msvc-14.2

    msbuild ALL_BUILD.vcxproj /p:Configuration=Release

The above commands should produce results in `build/src/Release` folder.

Library linking on Windows
--------------------------

Here is an example how to link code samples from this document assuming
the `libmkeclient` is built using above steps and the c++ code is stored
in `example.cpp` file.

In `x64 Native Tools Command Prompt for VS 2019` go to the
`libmkeclient` code base root folder (`mkeclient`) and execute the
following commands:

    SET MKECLIENT_SRC="src"
    SET MKECLIENT_LIB="build/src/Release/mkeclient.lib"
    SET MKEAPI_INCLUDE="../../../mkeapi/include"
    cl /MT /EHsc /I%MKECLIENT_SRC% /I%MKEAPI_INCLUDE% /I C:\local\boost_1_74_0 example.cpp /link %MKECLIENT_LIB% /MACHINE:X64 /LIBPATH:C:\local\boost_1_74_0\lib64-msvc-14.2\

The above commands should produce `example.exe` binary that can be run
via `example.exe` command. Paths in above commands assume default folder
structure from MkE SDK. If you want to build from elsewhere please
update the paths accordingly.

Connecting to the Sensor
========================

The `Client` class does not implement the connection to the MkE API
server directly. Rather, it uses the `BaseBus` class to manage the
connection. The `BaseBus` class has two child classes, `TcpBus` and
`SerialBus`, implementing TCP/IP and serial port style connections,
respectively. The next code snippet shows how to connect to a sensor
listening at `192.168.0.1:8888` through the TCP/IP network:

``` c++
#include "mke/cli/client.h"
#include "mke/cli/basebus.h"

int main(int argc, char* argv[]) {
  TcpBus tcp("192.168.0.1", 8888);
  Client client(&tcp);
  std::vector<char> buffer;
  client.setPayloadBuffer(&buffer);
  client.connect();
}
```

MkE API Constants, Errors and Data Structures
=============================================

All MkE constants, errors and data structures are defined in
`includes/mkeapi.h`

Note that the `Client` class does not perform any validity check of the
parameters. Rather, it lets the MkE API server decide, if the parameter
is valid or not.

Synchronous and Asynchronous Connection Methods
===============================================

There are two possible ways to work with `Client`

-   by using methods with synchronous interface (those with timeout
    parameter) and

-   by using methods with asynchronous interface (those with callback
    parameters).

The actual communication is always asynchronous, hence Client’s methods
with asynchronous interface provide more direct access to the MkE API
and can be faster in some cases (because response data can be given to
the user without an extra copying). On the other hand, Client’s methods
with synchronous interface are simpler to use. Client’s methods with
synchronous interface are just convenient wrappers of their asynchronous
equivalents. Both synchronous and asynchronous methods can be used
interchangeably and combined in one binary.

Making synchronous MkE API requests
===================================

The MkE API requests are implemented via the `Client` class methods,
please see [doxygen generated
documentation](html/classmke_1_1cli_1_1_client.html) for the full
reference of all available methods.

The most basic example of a MkE API request is the
`MKE_REQUEST_GET_STATE` request. This request is implemented by the
`get_state()` method:

``` c++
mke::api::MkEStateType state = client.getState();
```

The above code snippet expects a valid `Client` object connected to a
MkE API server. Note that since the call above is using synchronous
version of the request, it would not return immediately after sending
the request data packet. Rather, it will wait for the reply data packet,
check and parse the reply, and finally return the current state to user.

All synchronous methods can be given timeout parameter in milliseconds
to limit the maximum time waiting for a response, e.g.:

``` c++
mke::api::MkEStateType state = client.getState(500);
```

If the request fails from any reason (including reaching given timeout),
exception will be thrown. Example of error handling of synchronous call:

``` c++
try {
  mke::api::MkEStateType state = client.getState();
  std::cout << "State is: " << state << std::endl;
}
catch (mke::Error &e) {
  std::cout << "Error: " << e.what() << std::endl;
}
```

If needed, few types of exceptions can be distinguished in order to
customize error handling in different situations. For example this way:

``` c++
try {
  mke::api::MkEStateType state = client.getState();
  std::cout << "State is: " << state << std::endl;
}
catch (mke::cli::ServerFatalError &e) {
  std::cout << "Server fatal error: " << e.what() << std::endl;
}
catch (mke::cli::BadReplyError &e) {
  std::cout << "Bad reply error: " << e.what() << std::endl;
}
catch (mke::cli::IOError &e) {
  std::cout << "IO error: " << e.what() << std::endl;
}
catch (mke::Error &e) {
  std::cout << "Error: " << e.what() << std::endl;
}
```

More synchronous calls can share the same try-catch block. If
synchronous and asynchronous calls are both used in a code please note
that only synchronous calls throw exceptions. Asynchronous calls should
call error callbacks in case of an error, more on that below.

Making asynchronous MkE API requests
====================================

Code example of getting the device state using methods with asynchronous
interface:

``` c++
mke::cli::StateCallback stateCallback = [](mke::api::MkEStateType state) {
  std::cout << "State is: " << state << std::endl;
};
mke::cli::ErrorCallback errorCallback = [](const mke::Error& error) {
  std::cout << "Error happened: " << error.what() << std::endl;
};
client.getState(stateCallback, errorCallback);
```

The above code can be rewritten to use callbacks inline:

``` c++
client.getState(
  [](mke::api::MkEStateType state) {
    std::cout << "State is: " << state << std::endl;
  },
  [](const mke::Error& error) {
    std::cout << "Error happened: " << error.what() << std::endl;
  });
```

Client’s methods with asynchronous interface return immediately after
sending the request data packet. One of provided callbacks should be
called as soon as a result (or an error) is available.

Note: Since our trivial example above doesn’t have any kind of main
loop, one would need to add at least some sleep call (e.g.
`std::this_thread::sleep_for(std::chrono::seconds(1));` ), so the
program doesn’t end before callbacks are called.

For more detailed example on how to get device state, set device state
or how to retrieve other information about device, please refer to demo
`demo_device_state.cpp`:

Processing 3D Data
==================

The `Client` supports both ways of receiving 3D data frames, *i.e*,

-   by *client polling* via the `getFrame()` method and

-   by *sensor pushing* via the `startFramePush()` method.

For examples on how to get and process frames via these methods please
refer to demos `demo_getframe_sync.cpp`, `demo_getframe_async.cpp` and
`demo_pushframes.cpp`.

3rd party libraries integration
===============================

Client can be compiled with support for these 3rd party libraries:
Open3D, OpenCV or PCL. Please refer to `integration_examples` folder for
more information and examples.

Bibliography
============

-   \[\] *MagikEye API v1.0*, 2020, Magik Eye Inc.
