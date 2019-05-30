ip = "http://localhost:1234";

/**
 * 文件选择对话框打开之前调用事件初始化文件树
 */
function getFileTree(callback)
{
    var treedata=[];
    $.ajax({
        type: "GET",
        url: ip+"/datalist",
        dataType: "text",
        success: function(data){
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