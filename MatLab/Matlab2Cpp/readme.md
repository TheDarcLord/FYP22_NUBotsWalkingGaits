### **Structure of a C++ MEX Function**

#### **Prerequisites** 
>*Header files*:
<br> `mex.hpp` - C++ MEX API
<br> `mexAdapter.hpp` - MexFunctions class
<br> *Namespaces*:
<br> `matlab::mex` - MEX Interface
<br> `matlab::data` - MATLAB Data API
<br> `matlab::engine` - Engine API for C++

#### **Entry Point**
    class MexFunction : public matlab::mex::Function {
        public:
            void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
            // check input arguments
            // implement function
            ...
        }
    }

### ** Networking with C++ **
#### *Sockets*
Lets apps attach to the local network at different ports.
1. Server/Client Applications:
    1. *Client* App sends a request to a *Server* App
        1. Create **socket**
        2. **connect** to a server
        3. **send/recv** - loop until data recieve
        4. **shutdown** to end read/write
        5. **close** to release data ?
    2. *Server* App returns a reply
        1. Create **socket**
        2. **bind** to an address - Port
        3. **listen** on the Port, await connection
        4. **accept** the connection
        5. **send/recv** - similar to read/write files
        6. **shutdown** to end read/write
        7. **close** to release data ?

Port: 10013 ...