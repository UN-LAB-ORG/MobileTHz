#ifndef TCP_EXP_H
#define TCP_EXP_H

#include <QDateTime>
#include <QStringList>
#include <atomic>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using ClientID = int; // unique ID for each client connection

class tcpExp
{
public:
    tcpExp();
    ~tcpExp();

    enum class ConnectionStatus
    {
        Disconnected,
        Connecting,
        Connected
    };
    enum class MessageType
    {
        Update,
        SensorData,
        Command,
        Connection,
        Identify,
        Latency,
        Unknown
    };
    enum class Direction
    {
        Sent,
        Received
    };

    struct LogEntry
    {
        QString timestamp;
        MessageType type = MessageType::Unknown;
        QString type_str;
        Direction direction = Direction::Received;
        QString direction_str;
        QString role;
        QString content;
    };

    ConnectionStatus getConnectionStatus();
    QStringList getConnectedClients();
    std::vector<LogEntry> getAndClearLogQueue();

    // For the OBSERVER (server role)
    bool startServer(int port);

    // For the NODE roles (e.g., "UE", "AP")
    bool connectToServer(const std::string &ip_address, int port, const std::string &my_role);

    // Stops all networking activity and cleans up resources
    void stop();

    // --- Observer Methods ---
    // Sends data to a specific node by its role (e.g., "UE")
    bool sendDataToRole(const std::string &role, const std::string &data);
    // Broadcasts data to all connected and identified nodes
    bool broadcastData(const std::string &data);
    // Gets the role (e.g., "UE") associated with a client connection ID
    std::string getRoleForClientId(ClientID id);
    // Gets the number of currently connected clients
    size_t getConnectedClientCount();

    // --- Node Methods ---
    // Sends data from a Node to the Observer
    bool sendDataToServer(const std::string &data);

    // --- General Methods ---
    bool hasIncomingData();
    // For Observer: gets next packet and the ID of the client who sent it
    std::pair<ClientID, std::string> getNextPacket();
    // For Nodes: gets the next packet from the server
    std::string getNextPacketFromServer();
    void addLatencyLogEntry(const std::string &content);

private:
    void serverAcceptLoop();
    void clientReceiveLoop(ClientID client_id);
    void nodeReceiveLoop();
    void cleanupSocket(ClientID socket_fd);
    void addLogEntry(Direction direction,
                     const std::string &content,
                     const std::string &role_override = "");
    std::atomic<ConnectionStatus> connection_status_{ConnectionStatus::Disconnected};

    std::atomic<bool> is_running_{false};
    std::thread server_thread_;
    std::map<ClientID, std::thread> client_threads_;

    std::mutex queue_mutex_;
    std::queue<std::pair<ClientID, std::string>> incoming_packets_;

    std::mutex log_queue_mutex_;
    std::vector<LogEntry> log_queue_;

    std::mutex role_map_mutex_;
    std::map<ClientID, std::string> client_id_to_role_map_;
    std::map<std::string, ClientID> role_to_client_id_map_;

    std::chrono::steady_clock::time_point start_time_;

#ifdef _WIN32
    SOCKET listen_socket_ = INVALID_SOCKET;
    std::map<ClientID, SOCKET> client_sockets_;
    SOCKET server_socket_ = INVALID_SOCKET; // For the node connecting to the server
#else
    int listen_socket_ = -1;
    std::map<ClientID, int> client_sockets_;
    int server_socket_ = -1;
#endif
};

#endif
