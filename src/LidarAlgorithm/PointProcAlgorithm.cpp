#include"PointProcAlgorithm.h"
#include"../LidarGeometry/GeometryFlann.h"

long PointCloudSegment::PointCloudSegment_DBScan(Point3Ds pointSet, int *type, float knnRange)
{
	typedef PointCloudAdaptor<std::vector<Point3D>> PCAdaptor;
	const PCAdaptor pcAdaptorPnts(pointSet);

	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PCAdaptor>, PCAdaptor, 3> kd_tree;
	kd_tree treeIndex(3, pcAdaptorPnts, KDTreeSingleIndexAdaptorParams(10));
	treeIndex.buildIndex();

	memset(type, 0, sizeof(int)*pointSet.size());
	std::vector<Point3D> preparePntSet;
	preparePntSet.push_back(pointSet[0]);
	int typeNumber = 1;
	type[0] = typeNumber;


	for (int i = 0; i < pointSet.size(); ++i)
	{
		printf("%ld-%ld\r", pointSet.size(), i);
		//depth first
		while (!preparePntSet.empty()) {
			int vecLen = preparePntSet.size();
			double pnt[3] = { preparePntSet[vecLen - 1].x,preparePntSet[vecLen - 1].y,preparePntSet[vecLen - 1].z };
			preparePntSet.pop_back();
			std::vector<std::pair<size_t, double> > indices_dists;
			RadiusResultSet<double, size_t> resultSet((double)knnRange, indices_dists);
			//resultSet.init(ret_index, out_dist_sqr);
			treeIndex.findNeighbors(resultSet, &pnt[0], SearchParams());
			for (int j = 0; j < resultSet.m_indices_dists.size(); ++j)
			{
				int idxPnt = resultSet.m_indices_dists[j].first;
				if (type[idxPnt] == 0)
				{
					preparePntSet.push_back(pointSet[idxPnt]);
					type[idxPnt] = typeNumber;
				}
			}
		};
		if (type[i] != 0)
			continue;
		else
			preparePntSet.push_back(pointSet[i]);

		typeNumber++;
	}
	printf("get %d number of segment types\n", typeNumber - 1);
	return typeNumber - 1;
}