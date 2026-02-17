#include "tcpExp.h"
#include "Codebase/Software/jsonReader/jsonReader.hpp"
#include <iostream>

using json = nlohmann::json;

namespace
{
    QString messageTypeToString(tcpExp::MessageType type)
    {
        switch (type)
        {
        case tcpExp::MessageType::Update:
            return "UPDATE";
        case tcpExp::MessageType::SensorData:
            return "SENSOR_DATA";
        case tcpExp::MessageType::Command:
            return "COMMAND";
        case tcpExp::MessageType::Connection:
            return "CONNECTION";
        case tcpExp::MessageType::Identify:
            return "IDENTIFY";
        case tcpExp::MessageType::Latency:
            return "LATENCY";
        default:
            return "UNKNOWN";
        }
    }
} // namespace

tcpExp::tcpExp()
{
    start_time_ = std::chrono::steady_clock::now();
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        throw std::runtime_error("WSAStartup failed.");
    }
#endif
}

tcpExp::~tcpExp()
{
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

void tcpExp::stop()
{
    is_running_ = false;
    connection_status_.store(ConnectionStatus::Disconnected);

    // Close server listening socket to unblock accept()
#ifdef _WIN32
    if (listen_socket_ != INVALID_SOCKET)
        closesocket(listen_socket_);
    if (server_socket_ != INVALID_SOCKET)
        closesocket(server_socket_);
#else
    if (listen_socket_ != -1)
        close(listen_socket_);
    if (server_socket_ != -1)
        close(server_socket_);
#endif

    // Close all client sockets
    {
        std::lock_guard<std::mutex> lock(role_map_mutex_);
        for (auto const &[id, sock] : client_sockets_)
        {
            cleanupSocket(sock);
        }
        client_sockets_.clear();
    }

    if (server_thread_.joinable())
        server_thread_.join();
    for (auto &pair : client_threads_)
    {
        if (pair.second.joinable())
            pair.second.join();
    }
    client_threads_.clear();
}

void tcpExp::cleanupSocket(ClientID socket_fd)
{
#ifdef _WIN32
    closesocket(socket_fd);
#else
    close(socket_fd);
#endif
}

bool tcpExp::startServer(int port)
{
    connection_status_.store(ConnectionStatus::Connecting);
    listen_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef _WIN32
    if (listen_socket_ == INVALID_SOCKET)
    {
        std::cerr << "tcpExp: Failed to create server socket. Error: " << WSAGetLastError()
                  << std::endl;
        return false;
    }
#else
    if (listen_socket_ < 0)
    {
        perror("tcpExp: Failed to create server socket");
        return false;
    }
#endif

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(listen_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("tcpExp: Bind failed");
        stop();
        return false;
    }

    if (listen(listen_socket_, 5) < 0)
    {
        perror("tcpExp: Listen failed");
        stop();
        return false;
    }

    // After successful listen:
    is_running_ = true;
    server_thread_ = std::thread(&tcpExp::serverAcceptLoop, this);
    connection_status_.store(ConnectionStatus::Connected); // Server is "connected" once listening
    addLogEntry(Direction::Received, "Server started on port " + std::to_string(port));
    std::cout << "[tcpExp] Server started, listening on port " << port << std::endl;
    return true;
}

void tcpExp::serverAcceptLoop()
{
    while (is_running_)
    {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
#ifdef _WIN32
        SOCKET client_socket = accept(listen_socket_, (struct sockaddr *)&client_addr, &client_len);
        if (client_socket == INVALID_SOCKET)
        {
            if (is_running_)
                std::cerr << "tcpExp: Accept failed. Error: " << WSAGetLastError() << std::endl;
            continue;
        }
#else
        int client_socket = accept(listen_socket_, (struct sockaddr *)&client_addr, &client_len);
        if (client_socket < 0)
        {
            if (is_running_)
                perror("tcpExp: Accept failed");
            continue;
        }
#endif

        std::string client_ip = inet_ntoa(client_addr.sin_addr);
        addLogEntry(Direction::Received, "Accepted connection from " + client_ip);
        std::cout << "[tcpExp] Accepted connection from client " << client_socket << std::endl;
        {
            std::lock_guard<std::mutex> lock(role_map_mutex_);
            client_sockets_[client_socket] = client_socket;
        }
        client_threads_[client_socket] = std::thread(&tcpExp::clientReceiveLoop,
                                                     this,
                                                     client_socket);
    }
}

void tcpExp::clientReceiveLoop(ClientID client_id)
{
    char buffer[4096];
    std::string incomplete_packet;

    while (is_running_)
    {
        int bytes_received = recv(client_id, buffer, sizeof(buffer), 0);
        if (bytes_received > 0)
        {
            incomplete_packet.append(buffer, bytes_received);
            size_t pos;
            while ((pos = incomplete_packet.find('\n')) != std::string::npos)
            {
                std::string packet = incomplete_packet.substr(0, pos);
                incomplete_packet.erase(0, pos + 1);
                json msg = json::parse(packet, nullptr, false);

                if (msg.is_discarded())
                {
                    // Packet is not valid JSON. Log it as unknown but don't queue it for the engine.
                    addLogEntry(Direction::Received, packet, getRoleForClientId(client_id));
                    std::cerr << "Warning: Received non-JSON packet from client " << client_id
                              << std::endl;
                    continue; // Process the next packet in the buffer
                }

                // At this point, we have valid JSON.
                addLogEntry(Direction::Received, packet, getRoleForClientId(client_id));

                if (msg.value("type", "") == "IDENTIFY")
                {
                    // Handle the IDENTIFY message here, as it's a special control message.
                    std::string role = msg.value("role", "");
                    if (!role.empty())
                    {
                        std::lock_guard<std::mutex> lock(role_map_mutex_);
                        client_id_to_role_map_[client_id] = role;
                        role_to_client_id_map_[role] = client_id;
                        std::cout << "[tcpExp] Client " << client_id << " identified as: " << role
                                  << std::endl;
                    }
                }
                else
                {
                    // It's a valid JSON message but not IDENTIFY (e.g., SENSOR_DATA).
                    // This is data for the engine to process.
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    incoming_packets_.push({client_id, packet});
                }
            }
        }
        else
        {
            // Connection closed or error
            std::string role = getRoleForClientId(client_id);
            std::string log_msg = "Client " + std::to_string(client_id) + " (" + (role.empty() ? "unidentified" : role) + ") disconnected.";
            addLogEntry(Direction::Received, log_msg);
            std::cout << "[tcpExp] " << log_msg << std::endl;
            {
                std::lock_guard<std::mutex> lock(role_map_mutex_);
                client_sockets_.erase(client_id);
                client_id_to_role_map_.erase(client_id);
                if (!role.empty())
                    role_to_client_id_map_.erase(role);
            }
            break;
        }
    }
}

bool tcpExp::connectToServer(const std::string &ip_address, int port, const std::string &my_role)
{
    connection_status_.store(ConnectionStatus::Connecting);
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (server_socket_ == INVALID_SOCKET)
    {
        std::cerr << "tcpExp: Failed to create client socket." << std::endl;
        return false;
    }
#else
    if (server_socket_ < 0)
    {
        perror("tcpExp: Failed to create client socket");
        return false;
    }
#endif

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr);

    if (connect(server_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("tcpExp: Connection to server failed");
        connection_status_.store(ConnectionStatus::Disconnected);
        addLogEntry(Direction::Sent,
                    "Connection to " + ip_address + ":" + std::to_string(port) + " FAILED");
        return false;
    }

    is_running_ = true;
    server_thread_ = std::thread(&tcpExp::nodeReceiveLoop, this);
    connection_status_.store(ConnectionStatus::Connected);
    addLogEntry(Direction::Sent,
                "Connected to server at " + ip_address + ":" + std::to_string(port));
    std::cout << "[tcpExp] Connected to server at " << ip_address << ":" << port << std::endl;

    if (!my_role.empty())
    {
        json identify_msg;
        identify_msg["type"] = "IDENTIFY";
        identify_msg["role"] = my_role;
        sendDataToServer(identify_msg.dump() + "\n");
    }

    return true;
}

void tcpExp::nodeReceiveLoop()
{
    char buffer[4096];
    std::string incomplete_packet;

    while (is_running_)
    {
        int bytes_received = recv(server_socket_, buffer, sizeof(buffer), 0);
        if (bytes_received > 0)
        {
            incomplete_packet.append(buffer, bytes_received);
            size_t pos;
            while ((pos = incomplete_packet.find('\n')) != std::string::npos)
            {
                std::string packet = incomplete_packet.substr(0, pos);
                incomplete_packet.erase(0, pos + 1);
                addLogEntry(Direction::Received, packet);
                std::lock_guard<std::mutex> lock(queue_mutex_);
                incoming_packets_.push({server_socket_, packet});
            }
        }
        else
        {
            addLogEntry(Direction::Received, "Disconnected from server.");
            std::cout << "[tcpExp] Disconnected from server." << std::endl;
            connection_status_.store(ConnectionStatus::Disconnected);
            is_running_ = false;
            break;
        }
    }
}

bool tcpExp::sendDataToRole(const std::string &role, const std::string &data)
{
    addLogEntry(Direction::Sent, data, role);
    std::lock_guard<std::mutex> lock(role_map_mutex_);
    auto it = role_to_client_id_map_.find(role);
    if (it != role_to_client_id_map_.end())
    {
        ClientID client_id = it->second;
        send(client_id, data.c_str(), data.length(), 0);
        return true;
    }
    return false;
}

bool tcpExp::broadcastData(const std::string &data)
{
    // Assume broadcast is an Update
    addLogEntry(Direction::Sent, data, "ALL");
    std::lock_guard<std::mutex> lock(role_map_mutex_);
    for (const auto &pair : client_sockets_)
    {
        send(pair.second, data.c_str(), data.length(), 0);
    }
    return !client_sockets_.empty();
}

bool tcpExp::sendDataToServer(const std::string &data)
{
    // Assume data from a Node is SensorData
    addLogEntry(Direction::Sent, data, "Observer");
    if (send(server_socket_, data.c_str(), data.length(), 0) < 0)
    {
        perror("tcpExp: Failed to send data to server");
        return false;
    }
    return true;
}

bool tcpExp::hasIncomingData()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return !incoming_packets_.empty();
}

std::pair<ClientID, std::string> tcpExp::getNextPacket()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (incoming_packets_.empty())
        return {-1, ""};
    auto packet = incoming_packets_.front();
    incoming_packets_.pop();
    return packet;
}

std::string tcpExp::getNextPacketFromServer()
{
    while (is_running_)
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!incoming_packets_.empty())
            {
                auto packet = incoming_packets_.front();
                incoming_packets_.pop();
                return packet.second;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return "";
}

std::string tcpExp::getRoleForClientId(ClientID id)
{
    std::lock_guard<std::mutex> lock(role_map_mutex_);
    auto it = client_id_to_role_map_.find(id);
    if (it != client_id_to_role_map_.end())
    {
        return it->second;
    }
    return "";
}

size_t tcpExp::getConnectedClientCount()
{
    std::lock_guard<std::mutex> lock(role_map_mutex_);
    return client_sockets_.size();
}

tcpExp::ConnectionStatus tcpExp::getConnectionStatus()
{
    return connection_status_.load();
}

QStringList tcpExp::getConnectedClients()
{
    QStringList clients;
    std::lock_guard<std::mutex> lock(role_map_mutex_);
    for (const auto &pair : role_to_client_id_map_)
    {
        clients.append(QString::fromStdString(pair.first));
    }
    return clients;
}

std::vector<tcpExp::LogEntry> tcpExp::getAndClearLogQueue()
{
    std::vector<LogEntry> logs;
    std::lock_guard<std::mutex> lock(log_queue_mutex_);
    if (!log_queue_.empty())
    {
        logs.swap(log_queue_);
    }
    return logs;
}

void tcpExp::addLogEntry(Direction direction,
                         const std::string &content,
                         const std::string &role_override)
{
    // --- Calculate Elapsed Time ---
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

    // --- Format the elapsed time into HH:mm:ss.zzz ---
    long long total_seconds = elapsed_ms / 1000;
    int milliseconds = elapsed_ms % 1000;
    int seconds = total_seconds % 60;
    int total_minutes = total_seconds / 60;
    int minutes = total_minutes % 60;

    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << minutes << ":"
       << std::setw(2) << std::setfill('0') << seconds << "."
       << std::setw(3) << std::setfill('0') << milliseconds;

    LogEntry entry;
    entry.timestamp = QString::fromStdString(ss.str());
    entry.direction = direction;
    entry.direction_str = (direction == Direction::Sent) ? "-->" : "<--";
    entry.role = QString::fromStdString(role_override);

    json msg = json::parse(content, nullptr, false);
    if (msg.is_discarded() || !msg.contains("type"))
    {
        entry.type = MessageType::Unknown;
        entry.content = QString::fromStdString(content);
    }
    else
    {
        const std::string type_str = msg["type"].get<std::string>();
        if (type_str == "SENSOR_DATA")
        {
            entry.type = MessageType::SensorData;
            if (role_override.empty())
            {
                entry.role = QString::fromStdString(msg.value("role", ""));
            }

            QStringList summary_parts;

            if (msg.contains("payload") && msg["payload"].is_object())
            {
                const auto &payload = msg["payload"];

                // Helper lambdas
                auto fmtTriple = [](const json &obj,
                                    const char *a,
                                    const char *b,
                                    const char *c,
                                    int prec = 2) -> QString
                {
                    double ax = obj.value(a, 0.0);
                    double ay = obj.value(b, 0.0);
                    double az = obj.value(c, 0.0);
                    return QString("%1,%2,%3")
                        .arg(ax, 0, 'f', prec)
                        .arg(ay, 0, 'f', prec)
                        .arg(az, 0, 'f', prec);
                };
                auto fmtQuat = [](const json &q, int prec = 2) -> QString
                {
                    return QString("%1,%2,%3,%4")
                        .arg(q.value("w", 0.0), 0, 'f', prec)
                        .arg(q.value("x", 0.0), 0, 'f', prec)
                        .arg(q.value("y", 0.0), 0, 'f', prec)
                        .arg(q.value("z", 0.0), 0, 'f', prec);
                };
                auto fmtAngBlock = [](const json &obj, int precA = 1, int precB = 2) -> QString
                {
                    double angle = obj.value("angle", 0.0);
                    double vel = obj.value("velocity", 0.0);
                    double acc = obj.value("acceleration", 0.0);
                    return QString("ang:%1 vel:%2 acc:%3")
                        .arg(angle, 0, 'f', precA)
                        .arg(vel, 0, 'f', precB)
                        .arg(acc, 0, 'f', precB);
                };

                // IMU
                if (payload.contains("imu") && !payload["imu"].is_null() && payload["imu"].contains("data") && payload["imu"]["data"].is_object())
                {
                    const auto &imu_data = payload["imu"]["data"];
                    QStringList imu_parts;
                    if (imu_data.contains("acceleration") && imu_data["acceleration"].is_object())
                    {
                        imu_parts.push_back(QString("Acc[%1]")
                                                .arg(fmtTriple(imu_data["acceleration"], "x", "y", "z", 2)));
                    }
                    if (imu_data.contains("angular_velocity") && imu_data["angular_velocity"].is_object())
                    {
                        imu_parts.push_back(QString("Gyro[%1]")
                                                .arg(fmtTriple(imu_data["angular_velocity"], "x", "y", "z", 2)));
                    }
                    if (imu_data.contains("quaternion") && imu_data["quaternion"].is_object())
                    {
                        imu_parts.push_back(QString("Quat[%1]").arg(fmtQuat(imu_data["quaternion"], 2)));
                    }
                    if (!imu_parts.empty())
                    {
                        summary_parts.push_back("IMU:" + imu_parts.join(" "));
                    }
                }

                // Power
                if (payload.contains("power") && !payload["power"].is_null() && payload["power"].contains("data") && payload["power"]["data"].is_object())
                {
                    const auto &power_data = payload["power"]["data"];
                    if (power_data.contains("power_watts") && power_data["power_watts"].is_number())
                    {
                        double p_watts = power_data["power_watts"].get<double>();
                        double p_dbm = 10.0 * log10(std::max(p_watts, 1e-15) * 1000.0);
                        summary_parts.push_back(QString("Pwr:%1W (%2dBm)")
                                                    .arg(p_watts, 0, 'f', 6)
                                                    .arg(p_dbm, 0, 'f', 2));
                    }
                }

                // Rotary
                if (payload.contains("rotary") && !payload["rotary"].is_null() && payload["rotary"].contains("data") && payload["rotary"]["data"].is_object())
                {
                    const auto &rotary_data = payload["rotary"]["data"];
                    QStringList rot_parts;
                    if (rotary_data.contains("azimuth") && rotary_data["azimuth"].is_object())
                    {
                        rot_parts.push_back(QString("Az[%1]").arg(fmtAngBlock(rotary_data["azimuth"])));
                    }
                    if (rotary_data.contains("altitude") && rotary_data["altitude"].is_object())
                    {
                        rot_parts.push_back(QString("Alt[%1]").arg(fmtAngBlock(rotary_data["altitude"])));
                    }
                    if (rotary_data.contains("isMoving"))
                    {
                        int moving = 0;
                        try
                        {
                            moving = rotary_data["isMoving"].get<int>();
                        }
                        catch (...)
                        {
                        }
                        rot_parts.push_back(QString("Moving:%1").arg(moving));
                    }
                    if (!rot_parts.empty())
                    {
                        summary_parts.push_back("Rotary:" + rot_parts.join(" "));
                    }
                }
            }

            if (summary_parts.empty())
            {
                entry.content = "No payload data";
            }
            else
            {
                entry.content = summary_parts.join(" | ");
            }
        }
        else if (type_str == "UPDATE")
        {
            entry.type = MessageType::Update;
            entry.content = "Requesting sensor data from nodes.";
        }
        else if (type_str == "COMMAND")
        {
            entry.type = MessageType::Command;
            if (msg.contains("payload") && msg["payload"].is_object() && msg["payload"].contains("rotary_command"))
            {
                entry.content = QString::fromStdString(
                    msg["payload"]["rotary_command"].get<std::string>());
            }
            else
            {
                entry.content = "Malformed COMMAND_DATA";
            }
        }
        else if (type_str == "IDENTIFY")
        {
            entry.type = MessageType::Identify;
            entry.role = QString::fromStdString(msg.value("role", ""));
            entry.content = QString("Identifying as %1").arg(entry.role);
        }
        else
        {
            entry.type = MessageType::Unknown;
            entry.content = QString::fromStdString(content);
        }
    }

    entry.type_str = messageTypeToString(entry.type);

    std::lock_guard<std::mutex> lock(log_queue_mutex_);
    log_queue_.push_back(std::move(entry));
}

void tcpExp::addLatencyLogEntry(const std::string &content)
{
    // --- Calculate Elapsed Time ---
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

    // --- Format the elapsed time into HH:mm:ss.zzz ---
    long long total_seconds = elapsed_ms / 1000;
    int milliseconds = elapsed_ms % 1000;
    int seconds = total_seconds % 60;
    int total_minutes = total_seconds / 60;
    int minutes = total_minutes % 60;

    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << minutes << ":"
       << std::setw(2) << std::setfill('0') << seconds << "."
       << std::setw(3) << std::setfill('0') << milliseconds;

    LogEntry entry;
    entry.timestamp = QString::fromStdString(ss.str());
    entry.direction = Direction::Received; // Treat as an internal/received event
    entry.direction_str = "---";
    entry.role = "System";
    entry.type = MessageType::Latency;
    entry.type_str = messageTypeToString(entry.type);
    entry.content = QString::fromStdString(content);

    std::lock_guard<std::mutex> lock(log_queue_mutex_);
    log_queue_.push_back(std::move(entry));
}
