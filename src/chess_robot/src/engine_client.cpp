#include "engine_client.h"

#include <cerrno>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

using std::string;



EngineClient::EngineClient(string ip, int port_num) : IP(ip), port(port_num), sock(-1){
    
    sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(this->port);

    if(inet_pton(AF_INET, this->IP.c_str(), &serv_addr.sin_addr)<=0){
        std::cout << "Invalid IP" << std::endl;
        return;
    }

    if(::connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0){
        std::cout << "Connection failed. Error: " << strerror(errno) << std::endl;
        ::close(sock);
        sock = -1;
        return;
    }

    sendCommand("uci");
    waitForResponse("uciok");
}

EngineClient::~EngineClient(){
    if(sock != -1){
        sendCommand("quit");
        ::close(sock);
    }
}

void EngineClient::sendCommand(string command){
    command += "\n";
    send(sock,command.c_str(),command.length(), 0);
}



string EngineClient::readLine(){
    string response;
    char c;
    while(::recv(sock, &c, 1, 0) > 0){
        if(c == '\n') break;
        if(c == '\r') continue;
        response += c;
    }

    return response;
}

void EngineClient::waitForResponse(string expected){
    while(true) if(readLine().find(expected) != string::npos) break;
}


