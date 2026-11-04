
#ifndef ENGINE_CLIENT_HPP
#define ENGINE_CLIENT_HPP

#include <string>

class EngineClient{
private:

    std::string IP;
    int port;
    int sock;

public:

    // Constructor / Destructor
    EngineClient(std::string ip = "127.0.0.1", int port = 9000);
    ~EngineClient();

    // Sends command to engine
    void sendCommand(std::string command);

    // Reads line from socket
    std::string readLine();

    // Helper for init
    void waitForResponse(std::string expected);

    bool isConnected() const { return sock != -1; }
};


#endif // ENGINE_CLIENT_HPP