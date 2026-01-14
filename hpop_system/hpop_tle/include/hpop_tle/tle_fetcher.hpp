#pragma once

#include <string>
#include <vector>
#include <optional>
#include <functional>
#include <sstream>
#include <curl/curl.h>

#include "tle_parser.hpp"

namespace hpop_tle
{

/// Celestrak API endpoints
namespace celestrak
{
    // GP (General Perturbations) API endpoint
    constexpr const char* GP_API_BASE = "https://celestrak.org/NORAD/elements/gp.php";

    // Legacy TLE endpoint
    constexpr const char* TLE_API_BASE = "https://celestrak.org/NORAD/elements/";

    // Common catalog groups
    constexpr const char* STATIONS = "stations.txt";
    constexpr const char* ACTIVE = "active.txt";
    constexpr const char* STARLINK = "starlink.txt";
    constexpr const char* GPS = "gps-ops.txt";
    constexpr const char* GLONASS = "glo-ops.txt";
    constexpr const char* GALILEO = "galileo.txt";
    constexpr const char* BEIDOU = "beidou.txt";
}

/// Result of a TLE fetch operation
struct FetchResult
{
    bool success{false};
    std::string error_message;
    std::vector<TLEData> tles;
    int http_code{0};
};

/// TLE Fetcher class for Celestrak API
class TLEFetcher
{
public:
    TLEFetcher()
    {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }

    ~TLEFetcher()
    {
        curl_global_cleanup();
    }

    /// Fetch TLE by NORAD catalog ID
    FetchResult fetchByNoradId(uint32_t norad_id)
    {
        std::ostringstream url;
        url << celestrak::GP_API_BASE << "?CATNR=" << norad_id << "&FORMAT=TLE";
        return fetchFromUrl(url.str());
    }

    /// Fetch TLE by satellite name (partial match)
    FetchResult fetchByName(const std::string& name)
    {
        std::ostringstream url;
        url << celestrak::GP_API_BASE << "?NAME=" << urlEncode(name) << "&FORMAT=TLE";
        return fetchFromUrl(url.str());
    }

    /// Fetch TLE by international designator (e.g., "2020-001A")
    FetchResult fetchByIntlDesignator(const std::string& intl_des)
    {
        std::ostringstream url;
        url << celestrak::GP_API_BASE << "?INTDES=" << urlEncode(intl_des) << "&FORMAT=TLE";
        return fetchFromUrl(url.str());
    }

    /// Fetch multiple TLEs by NORAD IDs
    FetchResult fetchByNoradIds(const std::vector<uint32_t>& norad_ids)
    {
        if (norad_ids.empty())
        {
            return FetchResult{false, "No NORAD IDs provided", {}, 0};
        }

        // Celestrak supports comma-separated IDs
        std::ostringstream url;
        url << celestrak::GP_API_BASE << "?CATNR=";
        for (size_t i = 0; i < norad_ids.size(); ++i)
        {
            if (i > 0) url << ",";
            url << norad_ids[i];
        }
        url << "&FORMAT=TLE";
        return fetchFromUrl(url.str());
    }

    /// Fetch TLEs from a catalog group
    FetchResult fetchCatalog(const std::string& catalog)
    {
        std::string url = std::string(celestrak::TLE_API_BASE) + catalog;
        return fetchFromUrl(url);
    }

    /// Fetch space stations (ISS, Tiangong, etc.)
    FetchResult fetchSpaceStations()
    {
        return fetchCatalog(celestrak::STATIONS);
    }

    /// Fetch active satellites
    FetchResult fetchActiveSatellites()
    {
        return fetchCatalog(celestrak::ACTIVE);
    }

    /// Fetch Starlink constellation
    FetchResult fetchStarlink()
    {
        return fetchCatalog(celestrak::STARLINK);
    }

    /// Fetch GPS constellation
    FetchResult fetchGPS()
    {
        return fetchCatalog(celestrak::GPS);
    }

    /// Set timeout in seconds
    void setTimeout(long timeout_seconds)
    {
        timeout_ = timeout_seconds;
    }

    /// Set user agent
    void setUserAgent(const std::string& user_agent)
    {
        user_agent_ = user_agent;
    }

private:
    long timeout_{30};
    std::string user_agent_{"HPOP-ROS2/1.0"};

    /// CURL write callback
    static size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* userp)
    {
        size_t total_size = size * nmemb;
        userp->append(static_cast<char*>(contents), total_size);
        return total_size;
    }

    /// URL encode a string
    std::string urlEncode(const std::string& str)
    {
        CURL* curl = curl_easy_init();
        if (!curl) return str;

        char* encoded = curl_easy_escape(curl, str.c_str(), static_cast<int>(str.length()));
        std::string result = encoded ? encoded : str;
        if (encoded) curl_free(encoded);
        curl_easy_cleanup(curl);
        return result;
    }

    /// Fetch TLE data from URL
    FetchResult fetchFromUrl(const std::string& url)
    {
        FetchResult result;

        CURL* curl = curl_easy_init();
        if (!curl)
        {
            result.error_message = "Failed to initialize CURL";
            return result;
        }

        std::string response_body;

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, user_agent_.c_str());
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            result.error_message = curl_easy_strerror(res);
            curl_easy_cleanup(curl);
            return result;
        }

        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &result.http_code);
        curl_easy_cleanup(curl);

        if (result.http_code != 200)
        {
            result.error_message = "HTTP error: " + std::to_string(result.http_code);
            if (response_body.find("No GP data found") != std::string::npos)
            {
                result.error_message = "No TLE data found for the given query";
            }
            return result;
        }

        // Parse the response
        result.tles = TLEParser::parseString(response_body);

        if (result.tles.empty())
        {
            result.error_message = "No valid TLEs found in response";
            return result;
        }

        result.success = true;
        return result;
    }
};

/// TLE Cache for reducing API calls
class TLECache
{
public:
    struct CachedTLE
    {
        TLEData tle;
        std::chrono::steady_clock::time_point fetch_time;
    };

    /// Get cached TLE if not expired
    std::optional<TLEData> get(uint32_t norad_id, std::chrono::seconds max_age = std::chrono::hours(24))
    {
        auto it = cache_.find(norad_id);
        if (it == cache_.end()) return std::nullopt;

        auto age = std::chrono::steady_clock::now() - it->second.fetch_time;
        if (age > max_age) return std::nullopt;

        return it->second.tle;
    }

    /// Store TLE in cache
    void put(const TLEData& tle)
    {
        cache_[tle.norad_id] = CachedTLE{tle, std::chrono::steady_clock::now()};
    }

    /// Clear cache
    void clear()
    {
        cache_.clear();
    }

    /// Get cache size
    size_t size() const
    {
        return cache_.size();
    }

private:
    std::unordered_map<uint32_t, CachedTLE> cache_;
};

} // namespace hpop_tle
