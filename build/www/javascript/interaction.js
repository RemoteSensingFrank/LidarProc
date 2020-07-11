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

/**
 * 将WGS84经纬度转换为UTM投影坐标
 * @param ：纬度
 * @param ：经度
 * @param ：投影带
 */
function projectedFromWGS84(lat,lng,zone){
    // let pointcloudProjection = "+proj=utm +zone=20 +ellps=GRS80 +datum=NAD83 +units=m +no_defs";
    // let mapProjection = proj4.defs("WGS84");
    var xy = proj4('+proj=utm +zone='+zone+' +ellps=WGS84 +datum=WGS84 +units=m +no_defs',[lng,lat]);
    return [xy[0], xy[1]]
}

/**
 * 将UTM投影坐标转换为WGS84经纬度
 * @param ：纬度
 * @param ：经度
 * @param ：投影带
 */
function projectedToWGS84(x,y,zone){
    var lla=proj4('+proj=utm +zone=' +zone+' +ellps=WGS84 +datum=WGS84 +units=m +no_defs').inverse([x,y])
    return lla;
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
 * @param ：平差输出结果 
 */
function loadCameraPositions(reconstructions){
    unloadCameraPositions();
    var ncams = 0;
    for(var reconstruction_id=0;reconstruction_id<reconstructions.length;reconstruction_id++){
        for (var shot_id in reconstructions[reconstruction_id].shots) {
            ncams+=1;
        }
    }
    var imageobj=Array(ncams);
    var group = new THREE.Group();
    group.name="camera";
    var imagenum = 0;
    for(var reconstruction_id=0;reconstruction_id<reconstructions.length;reconstruction_id++){
        for (var shot_id in reconstructions[reconstruction_id].shots) {
            if (reconstructions[reconstruction_id].shots.hasOwnProperty(shot_id)) {
                imageobj[imagenum]=cameraLineGeo(reconstructions[reconstruction_id],shot_id);
                imageobj[imagenum].myimagenum = imagenum;
                imageobj[imagenum].isFiltered = false;
                group.add(imageobj[imagenum]);
            }
            imagenum+=1;
        }
    }


    // for(var imagenum=0;imagenum<ncams;imagenum++){
    //     imageobj[imagenum]=makeImageFrustrum(camRoll[imagenum],camPitch[imagenum],camYaw[imagenum],camX[imagenum],camY[imagenum],camZ[imagenum]);
    //     imageobj[imagenum].myimagenum = imagenum;
    //     imageobj[imagenum].isFiltered = false;
    //     group.add(imageobj[imagenum]);
    // }
    viewer.scene.scene.add(group);
    viewer.scene.view.position.set(imageobj[0].position.x,imageobj[0].position.y,imageobj[0].position.z);
}

/**
 *移除相机姿态参数
 */
function unloadCameraPositions(){
    var obj=viewer.scene.scene.getObjectByName("camera");
    viewer.scene.scene.remove(obj);
}


/**
 * 旋转操作，这里得旋转操作采用罗戈里格斯变换进行
 * 而不是采用欧拉角的方式，因此在实现上存在一定的
 * 的差异
 * @param：待旋转的向量
 * @param：旋转角度
 */
function rotate(vector, angleaxis) {
    var v = new THREE.Vector3(vector[0], vector[1], vector[2]);
    var axis = new THREE.Vector3(angleaxis[0],
                                 angleaxis[1],
                                 angleaxis[2]);
    var angle = axis.length();
    axis.normalize();
    var matrix = new THREE.Matrix4().makeRotationAxis(axis, angle);
    v.applyMatrix4(matrix);
    return v;
}

/**
 * 计算相机光心在世界坐标系中的坐标
 * @param : 成像的shot
 */
function opticalCenter(shot) {
    var angleaxis = [-shot.rotation[0],
                     -shot.rotation[1],
                     -shot.rotation[2]];

    var cent = shot.gps_position;

    var lng=$("#centerCameraLng")[0].value==""?0:$("#centerCameraLng")[0].value
    var lat=$("#centerCameraLat")[0].value==""?0:$("#centerCameraLat")[0].value


    var centWorldPlane=projectedFromWGS84(lat,lng,50);
    if(lng==0&&lat==0){
        centWorldPlane[0]=centWorldPlane[1]=0;
    }

    var centWorld = [
        centWorldPlane[0],
        centWorldPlane[1],
        0
    ];                 

    var Rt = rotate(shot.translation, angleaxis,centWorld);
    Rt.negate();

    Rt.x = Rt.x+centWorld[0];
    Rt.y = Rt.y+centWorld[1];
    Rt.z = Rt.z+centWorld[2];
    //var lla=projectedToWGS84(Rt.x,Rt.y);
    // Rt.x = lla[0];
    // Rt.y = lla[1];
    return Rt;
}

/**
 * 将影像点解算为直接坐标系下的点
 * @param : 成像的相机参数
 * @param : 成像的shot
 * @param : 在像平面上的x坐标
 * @param : 在像平面上的y坐标
 * @param : 缩放比例参数
 */
function pixelToVertex(cam, shot, u, v, scale) {
    // Projection model:
    // xc = R * x + t
    // u = focal * xc / zc
    // v = focal * yc / zc
    var focal = cam.focal || 0.3;
    var zc = scale;
    var xc = u / focal * zc;
    var yc = v / focal * zc;

    var xct = [xc - shot.translation[0],
               yc - shot.translation[1],
               zc - shot.translation[2]];


    var angleaxis = [-shot.rotation[0],
                     -shot.rotation[1],
                     -shot.rotation[2]];
    var cent = shot.gps_position;

    var lng=$("#centerCameraLng")[0].value==""?0:$("#centerCameraLng")[0].value
    var lat=$("#centerCameraLat")[0].value==""?0:$("#centerCameraLat")[0].value

    var centWorldPlane=projectedFromWGS84(lat,lng,50);
    if(lng==0&&lat==0){
        centWorldPlane[0]=centWorldPlane[1]=0;
    }

    var centWorld = [
        centWorldPlane[0],
        centWorldPlane[1],
        0
    ];
    var Rt = rotate(xct, angleaxis,centWorld);
    Rt.x = Rt.x+centWorld[0];
    Rt.y = Rt.y+centWorld[1];
    Rt.z = Rt.z+centWorld[2];
    //var lla=projectedToWGS84(Rt.x,Rt.y);
    // Rt.x = lla[0];
    // Rt.y = lla[1];
    return Rt
}

/**
 * 将相机的成像范围转换为实际空间中的坐标
 * @param : 相片重构参数
 * @param : 相片id
 */
function cameraLineGeo(reconstruction, shot_id) {
    var shot = reconstruction.shots[shot_id];
    var cam  = reconstruction.cameras[shot.camera];

    var dx = cam.width / 2.0 / Math.max(cam.width, cam.height);
    var dy = cam.height / 2.0 / Math.max(cam.width, cam.height);

    var pyramidgeometry = new THREE.Geometry();
    var imagepyramid  = new THREE.Object3D();

    pyramidgeometry.vertices = [
        pixelToVertex(cam, shot, -dx, -dy, 0.9),
        pixelToVertex(cam, shot,  dx, -dy, 0.9),
        pixelToVertex(cam, shot,  dx,  dy, 0.9),
        pixelToVertex(cam, shot, -dx,  dy, 0.9),
        opticalCenter(shot)
    ];

    pyramidgeometry.faces = [
        new THREE.Face3( 1, 0, 4 ),
        new THREE.Face3( 2, 1, 4 ),
        new THREE.Face3( 3, 2, 4 ),
        new THREE.Face3( 0, 3, 4 )
    ];

    var pyramidmaterial = new THREE.MeshBasicMaterial( {   color: 0xff0000,
        wireframe: false
    } );
    var lng=$("#centerCameraLng")[0].value==""?0:$("#centerCameraLng")[0].value
    var lat=$("#centerCameraLat")[0].value==""?0:$("#centerCameraLat")[0].value


    var centWorldPlane=projectedFromWGS84(lat,lng,50);
    if(lng==0&&lat==0){
        centWorldPlane[0]=centWorldPlane[1]=0;
    }
    imagepyramid.position.x = pyramidgeometry.vertices[4].x;
    imagepyramid.position.y = pyramidgeometry.vertices[4].y;
    imagepyramid.position.z = pyramidgeometry.vertices[4].z;

    // imagepyramid.rotation.x = -shot.rotation[0];
    // imagepyramid.rotation.y = -shot.rotation[1];
    // imagepyramid.rotation.z = -shot.rotation[2];

    // imagepyramid.scale.x = 1;
    // imagepyramid.scale.y = 1;
    // imagepyramid.scale.z = 1;
    var pyramid = new THREE.Mesh( pyramidgeometry, pyramidmaterial );
    imagepyramid.add(pyramid);
    return imagepyramid
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