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
        var selectedNodes = $("#treeDeleteData").treeview("getSelected");
        var parentNodes = $("#treeDeleteData").treeview('getParent', selectedNodes[0]);
        var filename=parentNodes.text+"/"+selectedNodes[0].text;
        deleteDataFile(filename);
    }
}

/**
 * 杆塔提取
 */
function extractTower(){
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
    }

}