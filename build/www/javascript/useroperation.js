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
    
    const query = new AV.Query('laslink');
    query.contains('LASPATH', selectedNodes[0].text)
    query.find().then((objects)=>{
        if(objects.length==0){
            alert("注意，无对应LAS文件，无法进行对应操作！");
        }
    });

    //first remove all the data
    // viewer.scene.pointclouds.forEach(function(layer) {
    //     window.viewer.scene.scenePointCloud.remove(layer);
    // });
    // viewer.scene.pointclouds = [];
    
    //remove tree node
    let pcRoot = $("#jstree_scene").jstree().get_json("pointclouds");
    var childNodes = $("#jstree_scene").jstree().get_children_dom(pcRoot);
    for (var i = 0; i < childNodes.length; i++) {
        $("#jstree_scene").jstree("delete_node", childNodes[i].id)
    }



    //remove map annotation
    // viewer.mapView.getSourcesLabelLayer().getSource().clear();
    // viewer.mapView.getSourcesLayer().getSource().clear();
    // viewer.mapView.getAnnotationsLayer().getSource().clear();
    // viewer.mapView.getExtentsLayer().getSource().clear();
    //CesiumInitial('https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer');

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
 * 线拟合优化
 */
function lineRefine(){
    var selectedNodes = $("#treeRefine").treeview("getSelected");
    var parentNodes = $("#treeRefine").treeview('getParent', selectedNodes[0]);
    var filename=parentNodes.text+"/"+selectedNodes[0].text;
    viewer.scene.calculateFeature(filename);
    $('#lineRefine').modal('hide');
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
            var parentdataNodes = $("#treeDeleteData").treeview('getParent', selecteddataNodes[0]);
            var filename=parentdataNodes.text+"/"+selecteddataNodes[0].text;
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
            url: "/datalist",
            dataType: "json",
            async:true,
            beforeSend:function(XMLHttpRequest){ 
                $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
                $("#loading").show()
            }, 

            success: function(data){
                $("#loading").empty(); //ajax返回成功，清除loading图标
                $("#loading").hide();

                for(i=0;i<data["dirobjects"].length ;i++){
                    if(data["dirobjects"][i]["dir"]["dirname"]!=""){
                        for(j=0;j<data["dirobjects"][i]["dir"]["fileobjects"].length; j++){
                            if(data["dirobjects"][i]["dir"]["fileobjects"][j]["name"]!=""){
                                $("#classifiedFileList")[0].options.add(new Option(data["dirobjects"][i]["dir"]["fileobjects"][j]["name"],
                                                                                   data["dirobjects"][i]["dir"]["dirname"]));
                            }
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
    $("#cameraModel").modal({
        backdrop:"static",  //点击背景不关闭
        keyboard: false     //触发键盘esc事件时不关闭
    });
}

/**
 * 打开camera重建数据
 */
function onReconstructionFileSelected(evt) {
    let file = evt.target.files[0];
    $("#cameraModel").modal('hide');
    var reader = new FileReader();
    reader.onload = function(e) {
        data = JSON.parse(e.target.result);
        //setReconstructionData(data);
        loadCameraPositions(data)
    };
    reader.readAsText(file);
}

