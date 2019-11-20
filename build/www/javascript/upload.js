/*
 * @Descripttion: 文件上传
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-20 16:48:01
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2019-11-20 18:35:12
 */
uploader = WebUploader.create({
        // swf文件路径
        swf: 'http://localhost:1234/javascript/uilib/Uploader.swf',
        // 文件接收服务端。
        server: 'http://webuploader.duapp.com/server/fileupload.php',
        // 选择文件的按钮。可选。
        // 内部根据当前运行是创建，可能是input元素，也可能是flash.
        pick: '#picker',
        // 不压缩image, 默认如果是jpeg，文件上传前会压缩一把再上传！
        resize: false,
        auto: false
});
uploader.on('startUpload', function() { // 开始上传时，调用该方法
    console.log("start");
});
uploader.on("uploadBeforeSend",function(obj,data,header){
    console.log(uploadBeforeSend);
});
// 文件上传过程中创建进度条实时显示。
uploader.on('uploadProgress', function( file, percentage ) {
    console.log(file);
});
uploader.on( 'uploadSuccess', function( file ) {
    console.log("uploadSuccess");
});
uploader.on( 'uploadError', function( file ) {
    console.log("uploadError");
});

uploader.on( 'uploadComplete', function( file ) {
    console.log("uploadComplete");
});

function uploadData(){
    var task_id = WebUploader.Base.guid();
    uploader.option("formData",{
        task_id: task_id
    })
    try{
        uploader.upload();
    }catch(err){
        console.log(err);
    }
}