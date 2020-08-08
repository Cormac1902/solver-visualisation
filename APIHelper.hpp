//
// Created by cormac on 08/08/2020.
//

#ifndef INC_3DVIS_APIHELPER_HPP
#define INC_3DVIS_APIHELPER_HPP

#include <string>
#include <zmq.hpp>
#include "msgpack.hpp"

class APIHelper {
public:
    static std::string port_string(unsigned port) {
        return prepend_string("tcp://*:", port);
    }

    static std::string prepend_string(const string& str, unsigned port) {
        return str + std::to_string(port);
    }

    template<typename T>
    static T unpack(zmq::message_t &message) {
        msgpack::sbuffer buffer;
        buffer.write(static_cast<const char *>(message.data()), message.size());

        msgpack::object_handle result;
        msgpack::unpack(result, buffer.data(), buffer.size());

        return result.get().as<T>();
    }

};


#endif //INC_3DVIS_APIHELPER_HPP
