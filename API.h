//
// Created by cormac on 30/06/2020.
//
#include <zmq.hpp>
#include "msgpack.hpp"
#include "Display.h"
#include "APIHelper.hpp"

#ifndef INC_3DVIS_API_H
#define INC_3DVIS_API_H
#endif //INC_3DVIS_API_H

class Display;

class API {
private:
    zmq::context_t context;
    zmq::socket_t add_clause_socket,
            remove_clause_socket,
            variable_assignment_socket,
            variable_activity_socket,
            render_socket_push,
            change_graph_socket;

    Display &display;
    unsigned int graph_level;

    static unsigned ADD_CLAUSE_SOCKET;
    static unsigned REMOVE_CLAUSE_SOCKET;
    static unsigned VARIABLE_ASSIGNMENT_SOCKET;
    static unsigned VARIABLE_ACTIVITY_SOCKET;

    static inline std::vector<long> unpack_vector(zmq::message_t &message) {
        return APIHelper::unpack<std::vector<long>>(message);
    }

    static void send_unsigned(unsigned int x, zmq::socket_t &socket);

    inline void send_render(Display::RENDER_ENUM opt) {
        send_unsigned(static_cast<unsigned int>(opt), render_socket_push);
    }

    inline void send_edges_render() { send_render(Display::EDGES_UPDATE); }

    inline void send_vertices_render() { send_render(Display::VERTICES_UPDATE); }

    inline void send_change_graph() { send_render(Display::CHANGE_GRAPH); }

public:
    explicit API(Display *displayPtr) : context(1),
                                        add_clause_socket(context, ZMQ_PULL),
                                        remove_clause_socket(context, ZMQ_PULL),
                                        variable_assignment_socket(context, ZMQ_PULL),
                                        variable_activity_socket(context, ZMQ_PULL),
                                        render_socket_push(context, ZMQ_PUSH),
                                        change_graph_socket(context, ZMQ_REP),
                                        display(*displayPtr),
                                        graph_level(display.graphStackSize()) {
        display.getInteraction().setAPI(this);
        APIHelper::bind(add_clause_socket, ADD_CLAUSE_SOCKET);
        APIHelper::bind(remove_clause_socket, REMOVE_CLAUSE_SOCKET);
        APIHelper::bind(variable_assignment_socket, VARIABLE_ASSIGNMENT_SOCKET);
        APIHelper::bind(variable_activity_socket, VARIABLE_ACTIVITY_SOCKET);
        APIHelper::connect(render_socket_push, Display::RENDER_SOCKET);
        APIHelper::bind(change_graph_socket, Display::CHANGE_GRAPH_SOCKET);
        /*add_clause_socket.bind(APIHelper::bind_string(ADD_CLAUSE_SOCKET));
        remove_clause_socket.bind(APIHelper::bind_string(REMOVE_CLAUSE_SOCKET));
        variable_assignment_socket.bind(APIHelper::bind_string(VARIABLE_ASSIGNMENT_SOCKET));
        variable_activity_socket.bind(APIHelper::bind_string(VARIABLE_ACTIVITY_SOCKET));
        render_socket_push.connect(APIHelper::connect_string(Display::RENDER_SOCKET));
        change_graph_socket.bind(APIHelper::bind_string(Display::CHANGE_GRAPH_SOCKET));*/
    }

    ~API() {
        zmq_close(&add_clause_socket);
        zmq_close(&remove_clause_socket);
        zmq_close(&variable_assignment_socket);
        zmq_close(&variable_activity_socket);
        zmq_close(&render_socket_push);
        zmq_close(&change_graph_socket);
        zmq_ctx_destroy(&context);
    }

    void setGraphLevel(unsigned int graphLevel) {
        graph_level = graphLevel;
        send_change_graph();
    }

    [[noreturn]] void run_add_clause_socket();

    [[noreturn]] void run_remove_clause_socket();

    [[noreturn]] void run_assign_variable_truth_value();

    [[noreturn]] void run_increase_variable_activity_socket();

    [[noreturn]] void run_change_graph_socket();

    inline void send_start_interactor() { send_render(Display::START_INTERACTOR); }

    inline void send_stop_interactor() { send_render(Display::STOP_INTERACTOR); }

    void run();
};
