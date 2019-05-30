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