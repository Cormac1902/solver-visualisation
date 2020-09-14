#include <vector>
#include "string"
#include "iostream"

#include <zmq.hpp>
#include <msgpack.hpp>

namespace s_vis_zmq {
    class SVisZMQ {
    private:
        zmq::context_t context;
        zmq::socket_t add_socket,
                remove_socket,
                variable_assignment_socket,
                variable_activity_socket,
                api_running_socket,
                render_socket;

        static const unsigned start_interactor = 2;

        static void send(msgpack::sbuffer *buffer, zmq::socket_t &socket) {
            zmq::message_t request(buffer->size());

            memcpy(request.data(), buffer->data(), buffer->size());

            try {
                socket.send(request, zmq::send_flags::none);
            } catch (zmq::error_t) {
                std::cout << "Error" << std::endl;
            }
        }

        static void send_digit(unsigned digit, zmq::socket_t &socket) {
            zmq::message_t request(1);
            memcpy(request.data(), std::to_string(digit).c_str(), 1);
            socket.send(request, zmq::send_flags::none);
        }

        void send_variable_activity(unsigned long variable) {
            auto buffer = new msgpack::sbuffer;
            std::pair<unsigned long, bool> var_activity = {variable, true};
            msgpack::pack(buffer, var_activity);

            send(buffer, variable_activity_socket);

            delete buffer;
        }

        void send_variable_assignment(long variable, bool undef) {
            auto buffer = new msgpack::sbuffer;
            std::pair<long, bool> var_undef = {variable, undef};
            msgpack::pack(buffer, var_undef);

            send(buffer, variable_assignment_socket);

            delete buffer;
        }

        static void send_clause(std::vector<long> clause, zmq::socket_t &socket) {
            auto buffer = new msgpack::sbuffer;
            msgpack::pack(buffer, clause);

            send(buffer, socket);

            delete buffer;
        }

    public:
        SVisZMQ() : context(1),
                    add_socket(context, ZMQ_PUSH),
                    remove_socket(context, ZMQ_PUSH),
                    variable_assignment_socket(context, ZMQ_PUSH),
                    variable_activity_socket(context, ZMQ_PUSH),
                    api_running_socket(context, ZMQ_REQ),
                    render_socket(context, ZMQ_PUSH) {
            api_running_socket.connect("tcp://localhost:29792");
            zmq::message_t request;
            send_digit(1, api_running_socket);
            api_running_socket.recv(request, zmq::recv_flags::none);
            zmq_close(&api_running_socket);

            add_socket.connect("tcp://localhost:29786");
            remove_socket.connect("tcp://localhost:29787");
            variable_assignment_socket.connect("tcp://localhost:29788");
            variable_activity_socket.connect("tcp://localhost:29789");
            render_socket.connect("tcp://localhost:29790");
        }

        ~SVisZMQ() {
            zmq_close(&add_socket);
            zmq_close(&remove_socket);
            zmq_close(&variable_assignment_socket);
            zmq_close(&variable_activity_socket);
            zmq_close(&render_socket);
            zmq_ctx_destroy(&context);
        }

        void add_clause(std::vector<long> clause) {
            send_clause(clause, add_socket);
        }

        void remove_clause(std::vector<long> clause) {
            send_clause(clause, remove_socket);
        }

        void assign_variable(long variable, bool undef = false) {
            send_variable_assignment(variable, undef);
        }

        void variable_activity(unsigned long variable) {
            send_variable_activity(variable);
        }

        void send_start_interactor() {
            send_digit(start_interactor, render_socket);
        }
    };
}
