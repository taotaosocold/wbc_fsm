#ifndef NPZ_READER_H
#define NPZ_READER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <regex>
#include <sys/stat.h>

class NPZReader
{
public:
    struct NPYArray
    {
        std::string name;
        std::vector<uint32_t> shape;
        std::vector<float> data_f32;
        std::vector<int64_t> data_i64;
        char dtype_code;  // 'f' for float32, 'i' for int64
        int dtype_bytes;

        bool empty() const
        {
            return data_f32.empty() && data_i64.empty();
        }
    };

    static bool loadNPZ(const std::string &npzPath,
                        std::vector<float> &joint_pos, std::vector<uint32_t> &joint_pos_shape,
                        std::vector<float> &joint_vel, std::vector<uint32_t> &joint_vel_shape,
                        std::vector<float> &body_pos_w, std::vector<uint32_t> &body_pos_w_shape,
                        std::vector<float> &body_quat_w, std::vector<uint32_t> &body_quat_w_shape,
                        std::vector<float> &body_ang_vel_w, std::vector<uint32_t> &body_ang_vel_w_shape,
                        std::vector<float> &body_lin_vel_w, std::vector<uint32_t> &body_lin_vel_w_shape,
                        std::vector<int64_t> &fps, std::vector<uint32_t> &fps_shape)
    {
        char tmpDir[] = "/tmp/npz_reader_XXXXXX";
        if (mkdtemp(tmpDir) == nullptr)
        {
            std::cerr << "[NPZReader] Failed to create temp dir" << std::endl;
            return false;
        }
        std::string tmpPath(tmpDir);

        std::string cmd = "unzip -o " + npzPath + " -d " + tmpPath + " > /dev/null 2>&1";
        int ret = std::system(cmd.c_str());
        if (ret != 0)
        {
            std::cerr << "[NPZReader] Failed to unzip: " << npzPath << std::endl;
            cleanup(tmpPath);
            return false;
        }

        bool ok = true;
        ok = ok && loadNPY(tmpPath + "/joint_pos.npy", "joint_pos", joint_pos, joint_pos_shape);
        ok = ok && loadNPY(tmpPath + "/joint_vel.npy", "joint_vel", joint_vel, joint_vel_shape);
        ok = ok && loadNPY(tmpPath + "/body_pos_w.npy", "body_pos_w", body_pos_w, body_pos_w_shape);
        ok = ok && loadNPY(tmpPath + "/body_quat_w.npy", "body_quat_w", body_quat_w, body_quat_w_shape);
        ok = ok && loadNPY(tmpPath + "/body_ang_vel_w.npy", "body_ang_vel_w", body_ang_vel_w, body_ang_vel_w_shape);
        ok = ok && loadNPY(tmpPath + "/body_lin_vel_w.npy", "body_lin_vel_w", body_lin_vel_w, body_lin_vel_w_shape);
        ok = ok && loadNPY_I64(tmpPath + "/fps.npy", "fps", fps, fps_shape);

        cleanup(tmpPath);
        return ok;
    }

private:
    static void cleanup(const std::string &dir)
    {
        std::string cmd = "rm -rf " + dir;
        std::system(cmd.c_str());
    }

    static bool loadNPY(const std::string &filePath, const std::string &name,
                        std::vector<float> &data, std::vector<uint32_t> &shape)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "[NPZReader] Cannot open: " << filePath << std::endl;
            return false;
        }

        if (!parseNPYHeader(file, shape))
        {
            std::cerr << "[NPZReader] Bad header in: " << filePath << std::endl;
            return false;
        }

        size_t total = 1;
        for (auto d : shape)
            total *= d;
        data.resize(total);
        file.read(reinterpret_cast<char *>(data.data()), total * sizeof(float));

        if (!file.good())
        {
            std::cerr << "[NPZReader] Error reading data: " << filePath << std::endl;
            return false;
        }

        std::cout << "[NPZReader] " << name << ": shape=(";
        for (size_t i = 0; i < shape.size(); i++)
        {
            if (i > 0)
                std::cout << ",";
            std::cout << shape[i];
        }
        std::cout << ")" << std::endl;

        return true;
    }

    static bool loadNPY_I64(const std::string &filePath, const std::string &name,
                            std::vector<int64_t> &data, std::vector<uint32_t> &shape)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "[NPZReader] Cannot open: " << filePath << std::endl;
            return false;
        }

        if (!parseNPYHeader(file, shape))
        {
            return false;
        }

        size_t total = 1;
        for (auto d : shape)
            total *= d;
        data.resize(total);
        file.read(reinterpret_cast<char *>(data.data()), total * sizeof(int64_t));

        std::cout << "[NPZReader] " << name << ": shape=(";
        for (size_t i = 0; i < shape.size(); i++)
        {
            if (i > 0)
                std::cout << ",";
            std::cout << shape[i];
        }
        std::cout << ")" << std::endl;

        return file.good();
    }

    static bool parseNPYHeader(std::ifstream &file, std::vector<uint32_t> &shape)
    {
        char magic[6];
        file.read(magic, 6);
        if (std::memcmp(magic, "\x93NUMPY", 6) != 0)
            return false;

        uint8_t major, minor;
        file.read(reinterpret_cast<char *>(&major), 1);
        file.read(reinterpret_cast<char *>(&minor), 1);

        uint16_t headerLen;
        file.read(reinterpret_cast<char *>(&headerLen), 2);

        std::vector<char> headerBuf(headerLen + 1);
        file.read(headerBuf.data(), headerLen);
        headerBuf[headerLen] = '\0';
        std::string header(headerBuf.data());

        shape.clear();
        size_t pos = header.find("'shape':");
        if (pos == std::string::npos)
            return false;

        pos = header.find('(', pos);
        if (pos == std::string::npos)
            return false;

        size_t end = header.find(')', pos);
        if (end == std::string::npos)
            return false;

        std::string shapeStr = header.substr(pos + 1, end - pos - 1);
        if (shapeStr.empty())
            return true;

        std::stringstream ss(shapeStr);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty())
                shape.push_back(static_cast<uint32_t>(std::stoul(token)));
        }

        return true;
    }
};

#endif // NPZ_READER_H
