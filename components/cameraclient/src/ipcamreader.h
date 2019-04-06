
#ifndef IPCAMREADER_H
#define IPCAMREADER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <chrono>

//#define URL "http://192.168.0.100:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00"
//#define URL "http://10.253.247.24:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00"
#define URL "http://pbustos:Zebulon00@192.168.1.104:20004/videostream.cgi"

static std::queue<cv::Mat> micola;

class IPCamReader
{
    public:
        ~IPCamReader()
        {
            curl_easy_cleanup(curl_handle);
            free(chunk.memory);
            curl_global_cleanup();
        }
        std::tuple<bool, cv::Mat> read() const 
        { 
            if(micola.empty() == false)
            {
                auto frame = micola.front(); 
                micola.pop(); 
                return std::make_tuple(true, frame); 
            }
            else
                return std::make_tuple(false, cv::Mat());
        }

        bool empty() const { return micola.empty();};
        cv::Mat& front() { return micola.front();};
        void pop() { micola.pop();};
        void run()
        {
            t = std::thread(&IPCamReader::init, this);
        }
	    void init()
        {
            
            curl_global_init(CURL_GLOBAL_ALL);
            curl_handle = curl_easy_init();
            curl_easy_setopt(curl_handle, CURLOPT_URL, URL);
            curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
            curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &chunk);
            curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
            res = curl_easy_perform(curl_handle);
            if (res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            else
                printf("%lu bytes retrieved\n", (unsigned long)chunk.size);
        }
    private:
        std::thread t;
        struct MemoryStruct
        {
            char *memory;
            size_t size=0;
            size_t begin=0, end=0;
            size_t init=0;
        };
        MemoryStruct chunk{(char *)malloc(1), 0, 0, 0};
        CURL *curl_handle;
        CURLcode res;

        static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, MemoryStruct *mem)
        {
            static auto begin = std::chrono::steady_clock::now();
            size_t realsize = size * nmemb;
            size_t current = mem->size;
            mem->memory  = (char *)realloc(mem->memory, mem->size + realsize + 1);
            mem->size += realsize;
            if (mem->memory == nullptr)
            {
                printf("WriteMemoryCallback: Not enough memory (realloc returned NULL)\n");
                return 0;
            }
            memcpy(mem->memory + current, contents, realsize);
            if (mem->size < 1024)
                return realsize;

            if (mem->begin == 0)  //search begin of frame
            {
                for (size_t i = mem->init; i < mem->size - 1; ++i)
                    if ((unsigned char)mem->memory[i] == 0xFF && (unsigned char)mem->memory[i + 1] == 0xD8)
                        mem->begin = i;
                if(mem->begin == 0)
                    mem->init = mem->size;
            }
            else // search end of frame 
            {   
                for(size_t j = 0; j < realsize; ++j)
                    if (*((unsigned char *)contents + j) == 0xff && *((unsigned char *)contents + j + 1) == 0xD9)
                        mem->end = current + j + 2;

                if (mem->end == 0)  //not found
                    mem->init = mem->size;
                else  //found
                {
                    std::vector<uchar> buf;
                    buf.assign(mem->memory + mem->begin, mem->memory + mem->end);
                    //cv::Mat img = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
                    //push into a fixed size queue
                    if( micola.size() < 2)
                        micola.emplace(cv::imdecode(buf, CV_LOAD_IMAGE_COLOR));

                    size_t nbytes = mem->size - mem->end;
                    char *tmp = (char *)malloc(nbytes);
                    memcpy(tmp, mem->memory + mem->end, nbytes);
                    free(mem->memory);
                    mem->size = 0;
                    mem->begin = mem->end = 0;
                    mem->memory = tmp;
                    // timing
                    auto end = std::chrono::steady_clock::now();
                    std::cout << "Elapsed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << std::endl;
                    begin = end;
                }
            }
            return realsize;
        }

};

#endif
