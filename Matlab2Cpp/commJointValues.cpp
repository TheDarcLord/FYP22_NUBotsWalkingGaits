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
            
            if(setSocketIPv4(mexIn2Str(inputs[0]),stoi(mexIn2Str(inputs[1]))) <= 0) {
                throwError("\nInvalid address || Address not supported\n");
            };

            const int sockFD = socket(server_address.sin_family, SOCK_STREAM, 0);
            if (sockFD < 0) {
                throwError("\nSocket creation error \n");
            };
            
            const int clientFD = connect(sockFD, (struct sockaddr*)&server_address, sizeof(server_address));
            if (clientFD < 0) {
                throwError("\nConnection Failed \n");
            };

            if(argc > 2) {
                std::string messageStr = createMessageStr(inputs[2]);
                char* messageChr = &messageStr[0];
                send(sockFD, messageChr, strlen(messageChr), 0);
                close(clientFD);
                close(sockFD);
            } else {
                char* messageChr = (char*) "GET";
                send(sockFD, messageChr, strlen(messageChr), 0);

                std::string reply(32, ' ');
                auto bytes_recv = recv(sockFD, &reply.front(), reply.size(), 0);
                if (bytes_recv == -1) {
                    std::cerr << "Error while receiving bytes\n";
                }

                std::cout << "\nClient recieved: " << reply << std::endl;
                close(sockFD);
                close(clientFD);
            }

            outputs[0] = inputs[0];
        }

    private:
        struct sockaddr_in server_address;

        /** 
         *  setSocketIPv4
         *      inet_pton() -> Converts IPv4 address from char[] to binary form
         */
        int setSocketIPv4(std::string ipAddressStr, int portNumber) {
            const char *IP_ADDRESS = ipAddressStr.c_str();
            const int pass = inet_pton(AF_INET, IP_ADDRESS, &server_address.sin_addr);
            server_address.sin_family = AF_INET;
            server_address.sin_port = htons(portNumber);
            return pass;
        }

        /** 
         *  createMessageStr
         */

        std::string createMessageStr(matlab::data::TypedArray<double> inArray) {
            std::ostringstream sstream;

            sstream.precision(4);
            sstream << " ";
            for(double value : inArray) {
                sstream << std::fixed << value << ", ";
            }
            std::string msg = sstream.str();
            msg.erase(msg.end()-2, msg.end());
            msg = msg + " ";

            return msg;
        }

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