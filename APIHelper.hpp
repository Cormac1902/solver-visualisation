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
    static std::string bind_string(unsigned port) {
        return prepend_string("tcp://*:", port);
    }

    static std::string connect_string(unsigned port) {
        return prepend_string("tcp://localhost:", port);
    }

    static std::string prepend_string(const std::string &str, unsigned port) {
        return str + std::to_string(port);
    }

    static void bind(zmq::socket_t &socket, unsigned port) {
        socket.bind(bind_string(port));
    }

    static void connect(zmq::socket_t &socket, unsigned port) {
        socket.connect(connect_string(port));
    }

    template<typename T>
    static inline T unpack(zmq::message_t &message) {
        msgpack::sbuffer buffer;
        buffer.write(static_cast<const char *>(message.data()), message.size());

        msgpack::object_handle result;
        msgpack::unpack(result, buffer.data(), buffer.size());

        return result.get().as<T>();
    }

    static inline long unpack_long(zmq::message_t &message) {
        try {
            return stol(message.to_string());
        } catch (std::invalid_argument &e) {
            return unpack<std::pair<long, bool>>(message).first;
        }
    }

};


#endif //INC_3DVIS_APIHELPER_HPP
