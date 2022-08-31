/*
 *
 *
 * 
 */

// Standard Headers
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
// Networking Headers
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// MatLab MEX specific headers
#include "mex.hpp"
#include "mexAdapter.hpp"

using namespace matlab::data;       // Use all Matlab Data Types
using matlab::mex::ArgumentList;    // 

class MexFunction : public matlab::mex::Function {

    public: 
    void operator()(ArgumentList outputs, ArgumentList inputs) {
        int argc = checkArguments(outputs, inputs);

        std::string ipAddressStr = mexIn2Str(inputs[0]);
        const char * IP_ADDRESS = ipAddressStr.c_str();
        int PORT = stoi(mexIn2Str(inputs[1]));

        int sock = 0, valread, client_fd, n;
        struct sockaddr_in serv_addr;
        char* hello = "Hello from client\n";

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            throwError("\n Socket creation error \n");
        }
    
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
    
        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, IP_ADDRESS, &serv_addr.sin_addr) <= 0) {
            throwError("\nInvalid address/ Address not supported \n");
        }
    
        if ((client_fd = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
            throwError("\nConnection Failed \n");
        }

        send(sock, hello, strlen(hello), 0);
        mprint("Hello message sent\n");

        std::string reply(32, ' ');
        auto bytes_recv = recv(sock, &reply.front(), reply.size(), 0);
        if (bytes_recv == -1) {
            std::cerr << "Error while receiving bytes\n";
        }

        std::cout << "\nClient recieved: " << reply << std::endl;
        close(sock);
        close(client_fd);

        outputs[0] = inputs[0];
    }

    private:

    /**
     *  checkArguments
     *      standard first in MEX function
     */ 
    int checkArguments(ArgumentList outputs, ArgumentList inputs) {
        if(inputs.size() > 3 || inputs.size() < 2) {
            mprint("Fetch Joint Variables: <IP_ADDRESS>, <PORT> \n");
            mprint("Send  Joint Variables: <IP_ADDRESS>, <PORT>, Joint_Variables[] \n");
            throwError("Incorrect number of arguments");
        }

        if( inputs[0].getType() != ArrayType::MATLAB_STRING || 
            inputs[1].getType() != ArrayType::MATLAB_STRING ) {
            throwError("IP Address and Port Number should be CHAR Arrays.\nUse Double Quotes\n");
        }

        if(inputs.size() > 2 && inputs[2].getType() != ArrayType::DOUBLE) {
            throwError("Joint Values should be...");
            // getDimensions(), getNumberOfElements(), etc ...
        }
        
        return inputs.size();
    }

    /**
     *  mprint()
     *      compacting the method of displaying MATLAB text
     *      ArrayFactory - Comes from `matlab::data`
     *                   - Generates `matlab::data::Array` objects
     * 
     *  Example:
     *      mprint( std::to_string() + " TEST" );
     */
    void mprint(std::string text) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        ArrayFactory factory;
        matlabPtr->feval("fprintf", 0, std::vector<Array>({ factory.createScalar(text) }));
    }

    /**
     *  throwError()
     *      compacting the method of throwing a MATLAB error
     *      ArrayFactory - Comes from `matlab::data`
     *                   - Generates `matlab::data::Array` objects
     */
    void throwError(std::string errorText) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        ArrayFactory factory; 
        matlabPtr->feval("error", 0, std::vector<Array>({ factory.createScalar(errorText) }));
    }
    
    /**
     *  mexIn2Str()
     *      pass in an object from the `ArgumentList[]` array
     *      return string
     */ 
    std::string mexIn2Str(matlab::data::TypedArray<matlab::data::MATLABString> input) {
        return input[0];
    }

};