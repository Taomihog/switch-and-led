// This file is not in use

#include <cstdint>
#include <optional>
#include <regex>
#include <string>

struct Params
{
    uint8_t N;
    uint8_t M;
    uint8_t V;
};

std::optional<Params> parse_filename(const std::string& filename)
{
    static const std::regex pattern(
        R"(^N([0-9]+)_M([0-9]+)_V([0-9]+)\.txt$)");

    std::smatch match;

    if (!std::regex_match(filename, match, pattern))
        return std::nullopt;

    try
    {
        unsigned long n = std::stoul(match[1].str());
        unsigned long m = std::stoul(match[2].str());
        unsigned long v = std::stoul(match[3].str());

        if (n > 255 || m > 255 || v > 255)
            return std::nullopt;

        return Params{
            static_cast<uint8_t>(n),
            static_cast<uint8_t>(m),
            static_cast<uint8_t>(v)
        };
    }
    catch (...)
    {
        return std::nullopt;
    }
}