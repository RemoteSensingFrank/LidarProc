/*
 * @Descripttion: 文件上传
 * @version: 1.0版本
 * @Author: Frank.Wu
 * @Date: 2019-11-20 16:48:01
 * @LastEditors: Frank.Wu
 * @LastEditTime: 2020-11-05 15:35:07
 */
$(document).ready(function() {
    var task_id = WebUploader.Base.guid(); // 产生文件唯一标识符task_id
    var uploader = WebUploader.create({
        swf: 'javascript/uilib/webuploader/Uploader.swf',
        server: '/upload', // 上传分片地址
        pick: '#picker',
        auto: true,
        chunked: true,
        chunkSize: 20 * 1024 * 1024,
        chunkRetry: 3,
        threads: 1,
        duplicate: true,
        formData: { // 上传分片的http请求中一同携带的数据
            task_id: task_id,
        },
    });

    uploader.on('startUpload', function() { // 开始上传时，调用该方法
        $('#progress').show();
        $('.progress-bar').css('width', '0%');
        $('.progress-bar').text('0%');
        $('.progress-bar').removeClass('progress-bar-danger progress-bar-success');
        $('.progress-bar').addClass('active progress-bar-striped');
    });

    uploader.on('uploadProgress', function(file, percentage) { // 一个分片上传成功后，调用该方法
        $('.progress-bar').css('width', percentage * 100 - 1 + '%');
        $('.progress-bar').text(Math.floor(percentage * 100 - 1) + '%');
    });

    uploader.on('uploadSuccess', function(file) { // 整个文件的所有分片都上传成功后，调用该方法
        var data = task_id+","+file.source['name'];
        $.ajax({
            type: "POST",
            url: "/upload-finish",
            contentType: "text/plain",
            data:data
        });
        //$.get('{{ url_for("upload_success") }}', data);
        $('.progress-bar').css('width', '100%');
        $('.progress-bar').text('100%');
        $('.progress-bar').addClass('progress-bar-success');
        $('.progress-bar').text('上传完成');
    });

    uploader.on('uploadError', function(file) { // 上传过程中发生异常，调用该方法
        $('.progress-bar').css('width', '100%');
        $('.progress-bar').text('100%');
        $('.progress-bar').addClass('progress-bar-danger');
        $('.progress-bar').text('上传失败');
    });

    uploader.on('uploadComplete', function(file) { // 上传结束，无论文件最终是否上传成功，该方法都会被调用
        $('.progress-bar').removeClass('active progress-bar-striped');
    });

    $('#progress').hide();
});