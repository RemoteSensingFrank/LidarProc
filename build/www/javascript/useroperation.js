/**
 * 这个文件主要是跟前端进行交互的js文件，主要是用户UI操作
 * Author：Frank.Wu
 * Date:2019-05-29
 * Version 1.0
 */

 
/**
 * 打开文件选择对话框
 */
function  initFileModal()
{
    getFileTree(function(){
        $("#fileModal").modal({
            backdrop:"static", //点击背景不关闭
            keyboard: false     //触发键盘esc事件时不关闭
        });
    });
}

/**
 * 打开文件转换对话框
 */
function initDataTransModal()
{
    getDataTransTree(function(){
        $("#transModal").modal({
            backdrop:"static", //点击背景不关闭
            keyboard: false     //触发键盘esc事件时不关闭
        });
    });
}

/**
 * 打开文件删除对话框
 */
function initDataDeleteModal()
{
    getDataDeleteTree(function(){
        $("#delModal").modal({
            backdrop:"static", //点击背景不关闭
            keyboard: false     //触发键盘esc事件时不关闭
        });
    });
}

/**
 * 打开文件上传对话框
 */
function initDataUploadModal()
{
    $("#uploadModal").modal({
        backdrop:"static", //点击背景不关闭
        keyboard: false     //触发键盘esc事件时不关闭
    });
}


/**
 * 根据选择的数据加载Potree的las文件
 */
function loadData()
{
    //获取选中节点名称
    var selectedNodes = $("#tree").treeview("getSelected");
    var path = "pointclouds/"+selectedNodes[0].text+"/cloud.js";
    
    //first remove all the data
    viewer.scene.pointclouds.forEach(function(layer) {
        window.viewer.scene.scenePointCloud.remove(layer);
    });
    viewer.scene.pointclouds = [];
    
    //remove tree node
    let pcRoot = $("#jstree_scene").jstree().get_json("pointclouds");
    var childNodes = $("#jstree_scene").jstree().get_children_dom(pcRoot);
    for (var i = 0; i < childNodes.length; i++) {

        $("#jstree_scene").jstree("delete_node", childNodes[i].id)
    }

    //remove map annotation
    viewer.mapView.getSourcesLabelLayer().getSource().clear();
    viewer.mapView.getSourcesLayer().getSource().clear();
    viewer.mapView.getAnnotationsLayer().getSource().clear();
    viewer.mapView.getExtentsLayer().getSource().clear();

    CesiumInitial('https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer');
    LoadLASDataViewer(path,"+proj=utm +zone=50 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");
    $('#fileModal').modal('hide');
}

/**
 * 转换选择的文件
 */
function transData(){
    var selectedNodes = $("#treeTrans").treeview("getSelected");
    var parentNodes = $("#treeTrans").treeview('getParent', selectedNodes[0]);
    var filename=parentNodes.text+"/"+selectedNodes[0].text;
    transDataFile(filename);
    $('#transModal').modal('hide');
}

/**
 * 删除选择文件
 */
function deleteData(){
    var res = confirm("确认删除所选文件？");
    if(res == true){

        //删除数据文件
        var selecteddataNodes = $("#treeDeleteData").treeview("getSelected");
        if(selecteddataNodes.length>0){
            var parentdataNodes = $("#treeDeleteData").treeview('getParent', selectedNodes[0]);
            var filename=parentNodes.text+"/"+selectedNodes[0].text;
            deleteDataFile(filename);
        }

        //删除展示文件
        var selectedexihibitNodes = $("#treeDeleteExhibit").treeview("getSelected");
        if(selectedexihibitNodes.length>0){
            var dirname = selectedexihibitNodes[0].text;
            deleteExhibitDirectory(dirname);
        }

    }
}

/**
 * 杆塔提取
 */
function classifiedFast(){
    var pntCount = 0;
    var measuresmens=viewer.scene.measurements;
    measuresmens.forEach(function(measureItem){
        if(measureItem.name="Point"){
            pntCount++;
        }
    });
    if(pntCount==0){
        $("#msgModal").modal({
            backdrop:"static",  //点击背景不关闭
            keyboard: false     //触发键盘esc事件时不关闭
        });
    }
    else{
        //extract 
        //选取了杆塔点
        //数据文件删除文件列表
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
                for(i=0;i<strs.length ;i++){
                    if(strs[i]!=""){
                        var fstr= new Array();
                        fstr=strs[i].split(";");
                        for(j=1;j<fstr.length; j++){
                            $("#classifiedFileList")[0].options.add(new Option(fstr[j],fstr[0]));
                        }               
                    }
                }
            },     
            error:function(data){
                console.log(data)
            }
        });
        $("#paramModal").modal({
            backdrop:"static",  //点击背景不关闭
            keyboard: false     //触发键盘esc事件时不关闭
        });

    }

}

/**
 * 加载相机信息
 */
function loadCamera(e){
    if($(e).hasClass('active')){  //判断是否选中
        unloadCameraPositions();
     }else{
        loadCameraPositions();
     }
     $(e).toggleClass('active');
}