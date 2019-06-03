/**
 *  这个文件主要是与后端交互的操作，从后端接口中获取数据
 * Author：Frank.Wu
 * Date:2019-05-29
 * Version 1.0
 */

ip = "http://localhost:1234";

/**
 * 文件选择对话框打开之前调用事件初始化文件树
 */
function getFileTree(callback)
{
    var treedata=[];
    $.ajax({
        type: "GET",
        url: ip+"/exhibitlist",
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show()
        }, 

        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();

            var strs= new Array();
            strs=data.split(";");
            var item={};
            item["text"]="数据文件";
            item["icon"]="glyphicon glyphicon-th-list",
            item["nodes"]=[];
            for(i=0;i<strs.length ;i++){
                if(strs[i]!=""){
                    var subitem={};
                    subitem["text"]=strs[i];
                    subitem["selectedIcon"]="glyphicon glyphicon-ok",
                    item["nodes"].push(subitem);
                }
            }
            treedata.push(item);
            $('#tree').treeview({
                data: treedata,         // data is not optional
                levels: 2,
                backColor:'white'
              });
            callback();
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 数据选择对话框打开之前调用初始化数据转换
 * @param {*} callback 
 */
function getDataTransTree(callback){
    var treedata=[];
    $.ajax({
        type: "GET",
        url: ip+"/datalist",
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show()
        }, 

        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();

            var strs= new Array();
            strs=data.split(",");

            var item={};
            item["text"]="数据文件";
            item["icon"]="glyphicon glyphicon-th-list",
            item["nodes"]=[];

            for(i=0;i<strs.length ;i++){
                if(strs[i]!=""){
                    var fstr= new Array();
                    fstr=strs[i].split(";");

                    var sitem={};
                    sitem["text"]=fstr[0];
                    sitem["icon"]="glyphicon glyphicon-th-list";
                    sitem["nodes"]=[];

                    for(j=1;j<fstr.length; j++){
                        if(fstr[j]!=""){
                            var subitem={};
                            subitem["text"]=fstr[j];
                            subitem["selectedIcon"]="glyphicon glyphicon-ok";
                            sitem["nodes"].push(subitem);
                        }
                    }
                    item["nodes"].push(sitem);
                }
            }
            treedata.push(item);
            $('#treeTrans').treeview({
                data: treedata,         // data is not optional
                levels: 2,
                backColor:'white'
              });
            callback();
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 数据选择对话框打开之前调用初始化数据删除
 * @param {*} callback 
 */
function getDataDeleteTree(callback){
    var treedata=[];
    $.ajax({
        type: "GET",
        url: ip+"/datalist",
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show()
        }, 

        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();

            var strs= new Array();
            strs=data.split(",");

            var item={};
            item["text"]="数据文件";
            item["icon"]="glyphicon glyphicon-th-list",
            item["nodes"]=[];

            for(i=0;i<strs.length ;i++){
                if(strs[i]!=""){
                    var fstr= new Array();
                    fstr=strs[i].split(";");

                    var sitem={};
                    sitem["text"]=fstr[0];
                    sitem["icon"]="glyphicon glyphicon-th-list";
                    sitem["nodes"]=[];

                    for(j=1;j<fstr.length; j++){
                        if(fstr[j]!=""){
                            var subitem={};
                            subitem["text"]=fstr[j];
                            subitem["selectedIcon"]="glyphicon glyphicon-ok";
                            sitem["nodes"].push(subitem);
                        }
                    }
                    item["nodes"].push(sitem);
                }
            }
            treedata.push(item);
            $('#treeDelete').treeview({
                data: treedata,         // data is not optional
                levels: 2,
                backColor:'white'
              });
            if(callback!=null)
                callback();
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 删除文件
 * @param {待删除文件文件名} filename 
 */
function deleteDataFile(filename){
    $.ajax({
        type: "GET",
        url: ip+"/datadelete/"+filename,
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show();
        }, 
        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();
            getDataDeleteTree(null);
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 将LAS文件转换为potree支持展示的格式
 * @param {文件名} filename 
 */
function transDataFile(filename){
    $.ajax({
        type: "GET",
        url: ip+"/datatrans/"+filename,
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show();
        }, 
        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();
        },     
        error:function(data){
            console.log(data)
        }
    });
}