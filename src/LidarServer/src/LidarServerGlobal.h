/**
    服务器还是想要构造为单例模式，以降低系统负载
**/
#include "httplib.h"
using namespace httplib;
class ServerSingle
{
public:
    static ServerSingle & getInstance()
    {
        static ServerSingle m_instance;
        return m_instance;
    }

private:
    Server* svr;
}