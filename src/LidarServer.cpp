// LidarProc.cpp: 定义控制台应用程序的入口点。
//
#include<stdio.h>
#include "./LidarServer/httplib.h"

int main(void)
{
    using namespace httplib;

    Server svr;

    svr.Get("/hi", [](const Request& req, Response& res) {

        res.set_content("Hello World!", "text/plain");
    });

    svr.Get(R"(/numbers/(\d+))", [&](const Request& req, Response& res) {
        auto numbers = req.matches[1];
        res.set_content(numbers, "text/plain");
    });
    svr.set_base_dir("./www");
    svr.listen("localhost", 1234);
}
