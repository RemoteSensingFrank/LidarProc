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
            $('#treeDeleteData').treeview({
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

    //展示数据文件删除文件列表
    var treeexhibit=[];
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
            item["text"]="展示文件";
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
            treeexhibit.push(item);
            $('#treeDeleteExhibit').treeview({
                data: treeexhibit,         // data is not optional
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
 * 删除数据文件
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

            //插入操作记录
            var operation = window.AV.Object.extend('operation');
            var op = new operation();
            op.save({
                operation_type:"delete data:"+filename,
                operation_ip:returnCitySN['cip']
            }).then(function(op) {
                // 成功
            }, function(error) {
                // 失败
                console.log(error);
            });
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 删除展示数据文件
 * @param {待删除的文件夹名称} dir 
 */
function deleteExhibitDirectory(dir){
    $.ajax({
        type: "GET",
        url: ip+"/exhibitdelete/"+dir,
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $("#loading").html("<img src='../resources/loading.svg'/>"); //在后台返回success之前显示loading图标
            $("#loading").show();
        }, 
        success: function(data){
            $("#loading").empty(); //ajax返回成功，清除loading图标
            $("#loading").hide();

            //插入操作记录
            var operation = window.AV.Object.extend('operation');
            var op = new operation();
            op.save({
                operation_type:"delete exhibit:"+dir,
                operation_ip:returnCitySN['cip']
            }).then(function(op) {
                // 成功
            }, function(error) {
                // 失败
                console.log(error);
            });

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
            //插入操作记录
            var operation = window.AV.Object.extend('operation');
            var op = new operation();
            op.save({
                operation_type:"trans to exhibit:"+filename,
                operation_ip:returnCitySN['cip']
            }).then(function(op) {
                // 成功
            }, function(error) {
                // 失败
                console.log(error);
            });            
        },     
        error:function(data){
            console.log(data)
        }
    });
}

/**
 * 点击上传按钮实现文件上传
 * 不需要点击上传按钮
 */
function uploadData(){
}

function projected2WGS84(x,y){
    let pointcloudProjection = "+proj=utm +zone=20 +ellps=GRS80 +datum=NAD83 +units=m +no_defs";
    let mapProjection = proj4.defs("WGS84");
    var lonlat = proj4(pointcloudProjection,mapProjection,[x,y]);
    return [lonlat[1], lonlat[0]]
}

function projectedFromWGS84(lat,lng){
    let pointcloudProjection = "+proj=utm +zone=20 +ellps=GRS80 +datum=NAD83 +units=m +no_defs";
    let mapProjection = proj4.defs("WGS84");

    var xy = proj4(mapProjection,pointcloudProjection,[lng,lat]);

    return [xy[0], xy[1]]
}

function projected2UTM(lat,lng){
    let pointcloudProjection = "+proj=utm +zone=20 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    let mapProjection = proj4.defs("WGS84");
    var xy = proj4(mapProjection,pointcloudProjection,[lng,lat]);
    return [xy[0], xy[1]]
}


/**
 * 根据相机情况创建相机的Frustrum并展示
 * @param {*} Rx 相机x轴旋转角度
 * @param {*} Ry 相机y轴旋转角度
 * @param {*} Rz 相机z轴旋转角度
 * @param {*} Cx 相机x方向平移距离
 * @param {*} Cy 相机y方向平移距离
 * @param {*} Cz 相机z方向平移距离
 */
function makeImageFrustrum(Rx,Ry,Rz,Cx,Cy,Cz){
    // instantiate a loader
    var loader = new THREE.TextureLoader();
    loader.crossOrigin = 'anonymous';
    //var imagetexture = loader.load(imagedir + imagename);

    var pixx = camPix[0]/camFocal;
    var pixy = camPix[1]/camFocal;

    var imageplane = new THREE.PlaneGeometry(pixx, pixy, 1, 1);
    imageplane.vertices[0].z = -1;
    imageplane.vertices[1].z = -1;
    imageplane.vertices[2].z = -1;
    imageplane.vertices[3].z = -1;

    var imagematerial = new THREE.MeshBasicMaterial( {
        color:0xeeefff,
        wireframe: true
    });
    var image = new THREE.Mesh(imageplane,imagematerial);
    var pyramidgeometry = new THREE.Geometry();

    pyramidgeometry.vertices = [
        new THREE.Vector3( -pixx/2, -pixy/2, -1 ),
        new THREE.Vector3( -pixx/2, pixy/2, -1 ),
        new THREE.Vector3( pixx/2, pixy/2, -1 ),
        new THREE.Vector3( pixx/2, -pixy/2, -1 ),
        new THREE.Vector3( 0, 0, 0 )
    ];

    pyramidgeometry.faces = [
        new THREE.Face3( 1, 0, 4 ),
        new THREE.Face3( 2, 1, 4 ),
        new THREE.Face3( 3, 2, 4 ),
        new THREE.Face3( 0, 3, 4 )
    ];

    var pyramidmaterial = new THREE.MeshBasicMaterial( {   color: 0xf8f9fa,
        wireframe: true
    } );


    var pyramid = new THREE.Mesh( pyramidgeometry, pyramidmaterial );

    var imagepyramid  = new THREE.Object3D();

    imagepyramid.add(image);
    imagepyramid.add(pyramid);

    let ptWgs84=projected2WGS84(Cx,Cy);
    let ptUTM = projected2UTM(ptWgs84[1],ptWgs84[0])
    imagepyramid.position.x = ptUTM[0];
    imagepyramid.position.y = ptUTM[1];
    imagepyramid.position.z = Cz+5000;

    imagepyramid.rotation.x = Rx * Math.PI/180;
    imagepyramid.rotation.y = Ry * Math.PI/180;
    imagepyramid.rotation.z = Rz * Math.PI/180;

    imagepyramid.scale.x = 1;
    imagepyramid.scale.y = 1;
    imagepyramid.scale.z = 1;

    return imagepyramid
}

/**
 * 加载或移除相机
 */
function loadCameraPositions(){
    var ncams = camX.length;
    var imageobj=Array(ncams);
    var group = new THREE.Group();
    group.name="camera";
    for(var imagenum=0;imagenum<ncams;imagenum++){
        imageobj[imagenum]=makeImageFrustrum(camRoll[imagenum],camPitch[imagenum],camYaw[imagenum],camX[imagenum],camY[imagenum],camZ[imagenum]);
        imageobj[imagenum].myimagenum = imagenum;
        imageobj[imagenum].isFiltered = false;
        group.add(imageobj[imagenum]);
    }
    viewer.scene.scene.add(group);
    viewer.scene.view.position.set(imageobj[0].position.x,imageobj[0].position.y,imageobj[0].position.z);
}

function unloadCameraPositions(){
    var obj=viewer.scene.scene.getObjectByName("camera");
    viewer.scene.scene.remove(obj);
}

/**
 * 点击分类处理进行分类
 */
function classifiedProc(){
    var towerRange=$("#towerRange")[0].value==""?15:$("#towerRange")[0].value;
    var lineHeight=$("#lineHeight")[0].value==""?7:$("#lineHeight")[0].value;
    var groundNetSize=$("#groundNetSize")[0].value==""?5:$("#groundNetSize")[0].value;
    var groundNetDis=$("#groundNetDis")[0].value==""?5:$("#groundNetDis")[0].value;
    var groundNetAngle=$("#groundNetAngle")[0].value==""?30:$("#groundNetAngle")[0].value;
    var vegeDistance=$("#vegeDistance")[0].value==""?15:$("#vegeDistance")[0].value;
    var classifiedName=$("#classifiedName")[0].value==""?"temp.las":$("#classifiedName")[0].value;
    var selects=$("#classifiedFileList")[0];
    var indexs = selects.selectedIndex;
    console.log(selects.options[indexs]);
    var srcFileName = selects.options[indexs].value+"-"+selects.options[indexs].text;
    if(srcFileName==undefined){
        console.log("未选择原始文件");
        return ;
    }

    var pntCount = 0;
    var measuresmens=viewer.scene.measurements;
    measuresmens.forEach(function(measureItem){
        if(measureItem.name="Point"){
            pntCount++;
        }
    });

    if(pntCount!=2)
    {
        console.log("杆塔数应该为两个表示一个档段");
        return;
    }
    var pointsParam="";
    measuresmens.forEach(function(measureItem){
        pointsParam+=measureItem.points[0].position.x+"-"+measureItem.points[0].position.y+"-";
    });
    pointsParam+=towerRange+"-"+lineHeight+"-"+groundNetSize+"-"+groundNetDis+"-"+groundNetAngle+"-"+vegeDistance+"-"+classifiedName+"-"+srcFileName;

    $.ajax({
        type: "GET",
        url: ip+"/classification/"+pointsParam,
        dataType: "text",
        async:true,
        beforeSend:function(XMLHttpRequest){ 
            $('#paramModal').modal('hide');
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