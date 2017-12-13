#ifndef ROSCPP_URL_H_
#define ROSCPP_URL_H_    

#include <string>

std::string uri_to_ip_address( const std::string uri );

bool is_uri_match( const std::string uri, const std::string ip_address );

class URLParser {
  public:
    URLParser(const std::string url_s); 
    ~URLParser( );

    bool match( const std::string ip_address );

    std::string get_host();

    inline std::string get_ip_address() { return ip_address_; }
    inline std::string get_port() { return port_; }
    inline std::string get_protocol() { return protocol_; }

  private:
    void parse(const std::string& url_s);

  private:
    std::string url_, protocol_, host_, path_, query_, ip_address_, port_;
};

#endif // ROSCPP_URL_H_ 
