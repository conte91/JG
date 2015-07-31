#include <iostream>
#include <boost/asio/ip/tcp.hpp>
#include <APC/APC.h>

int main(int argc, char** argv){
  boost::asio::ip::tcp::iostream connectedStream("www.google.com", "80");

  if(!connectedStream){
    std::cerr << "Couldn't connect\n";
  }
  connectedStream << "GET / HTTP/1.1\nHost: www.google.com\n\n";
  connectedStream.flush();
  do{
    std::string s;
    std::cout << "\nHAHAHAH\n";
    std::getline(connectedStream, s);
    std::cout << s;
    std::cout << "\nHBHBHBH\n";
  }while(connectedStream.rdbuf()->available());
  std::cout << std::endl;
  return 0;

}
